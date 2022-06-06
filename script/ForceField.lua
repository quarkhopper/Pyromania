#include "Utils.lua"

FF = {} -- library constants
FF.FORWARD = Vec(0, 0, -1)
FF.MAX_SIM_POINTS = 200

function inst_force_field_ff()
    -- create a force field instance.
    local inst = {}
    -- the tick counter allows certain activities to happen in a 
    -- staggered fashion rather than all at once on a single tick
    inst.tick_reset = 2
    inst.tick_count = inst.tick_reset

    -- The field is a hashed multidim array for fast location searching
    inst.field = {}
    -- the metafield is a lower resolution array that's an average of a 
    -- block of adjacent vectors. Can be operated on instead of the base
    -- field for performance, such as to drive effects that don't require
    -- high resolution.
    inst.metafield = {}
    -- These are "hit" events when a vector force tries to propagate into 
    -- a shape. This list is cleared before every propagation. This field is 
    -- regenerated regularly from the base field.
    inst.contacts = {}

    -- Resolution of the base field. How many world units per force vector. (actually the reverse of an 
    -- actual resolution  number)
    inst.resolution = 0.5
    -- Resolution of the metafield
    inst.meta_resolution = 2
    -- The maximum force assignable to a vector. It's clamped below this number.
    inst.f_max = 2
    -- The force magnitude below which a vector is culled from the field.
    inst.f_dead = 0.1
    -- The universal reduction (proportion) of magnitude of every vector in the field.
    inst.decay = 0.02
    -- The reduction of magnitude of a vector after it has propagated force to adjacent
    -- children
    inst.prop_decay  = 0.01
    -- Field extension propagation produces this many spokes radiating out from the parent
    -- vector into other field coordinates. That may combine with existing vectors in those
    -- coordinates or extend the field into new coordinates (adding to the field)
    inst.point_split = 6
    -- The angle in degrees that spokes are produced from the center parent vector when 
    -- propagating. 
    inst.extend_spread = 45

    return inst
end

function reset_ff(ff)
    ff.field = {}
    ff.metafield = {}
    ff.contacts = {}
end

function inst_field_contact(point, hit_point, normal, shape)
    -- A contact is a record of any time a force is propagated into a coordinates
    -- that is occupied by shape voxels. This is interpreted as a "hit" event by 
    -- the force. No new vector is created to occupy that coordinate, though if 
    -- a hole is created by a higher level process the force may be able to spread
    -- into the unnoccupied coordinate in the next tick.
    local inst = {}
    inst.point = point
    inst.hit_point = hit_point
    inst.normal = normal
    inst.shape = shape
    return inst
end

function inst_field_point(coord, resolution)
    -- One vector in the field. There's a static for setting either the position
    -- or the dir/mag combo to keep them consistent for efficiency so this doesn't
    -- have to be recalculated over and over. (I would love it if OOP was allowed in 
    -- the quickloaded code for the game so I could just make that a method...)
    local inst = {}
    inst.resolution = resolution
    inst.coord = coord
    local half_res = resolution/2
    inst.pos = VecAdd(VecScale(coord, inst.resolution), VecScale(Vec(1,1,1), half_res))
    inst.dir = Vec(0,1,0)
    inst.mag = 1
    inst.vec = Vec(0,1,0)
    inst.edge = false
    inst.type = point_type.base
    return inst
end

function set_point_vec(point, vec)
    -- Sets the force vector of a point and updates the 
    -- dir/mag attributes
    point.vec = vec
    point.dir = VecNormalize(vec)
    point.mag = VecLength(vec)
end

function set_point_dir_mag(point, dir, mag)
    -- sets the dir/mag of the field point and updates the
    -- vector
    point.dir = dir
    point.mag = mag
    point.vec = VecScale(dir, mag)
end

function apply_force(ff, pos, force)
    -- This is an interface function that sparks a force propagation through
    -- through the field. 
    local coord = pos_to_coord(pos, ff.resolution)
    local point = field_get(ff.field, coord)
    -- if this point doesn't exist yet in the coord of the field, we
    -- have a couple things to do
    if point == nil then 
        point = inst_field_point(coord, ff.resolution)
        -- set to true so we know on the next extend cycle to treat this as 
        -- an edge to extend into other field coordinates.
        point.edge = true
        -- insert a point into the field
        field_put(ff.field, point, point.coord)
    end
    -- if the point already exists, just override the point vector that's there
    set_point_vec(point, VecAdd(point.vec, force))
end

function extend_field(ff)
    -- called in the proper tick cycle to extend the field at the edges if the vector is pointing the right way. 
    ff.contacts = {}
    local points = flatten(ff.field)
    for i = 1, #points do
        local point = points[i]
        -- if the point was marked as an edge on the last cycle (it was brand new to the field last cycle)
        -- then we will look into extending it
        if point.edge then 
            -- extend the point itself in the direction of the existing vector
            extend_point(ff, point)
            -- find the direction of a number of spokes at a certain deflection angle
            -- away from the parent vector.
            local extend_dirs = radiate(point.vec, ff.extend_spread, ff.point_split, math.random() * 360)
            for i = 1, #extend_dirs do
                -- create a temporary point to act as the parent to the potential 
                -- extension.
                local extend_dir = extend_dirs[i]
                local temp_point = inst_field_point(point.coord, ff.resolution)
                set_point_vec(temp_point, VecScale(extend_dir, point.mag))
                extend_point(ff, temp_point)
            end   
            -- we've extended the point. It's no longer at the edge of the field. 
            point.edge = false 
        end
    end
end

function extend_point(ff, point, new_extensions)
    local extend_dir = point.dir
    local extend_coord = round_vec(VecAdd(point.coord, extend_dir))
    local extension_point = field_get(ff.field, extend_coord)
    -- only put a point where there isn't one already
    if extension_point == nil then 
        -- check if we're hitting something on the way to extending
        local hit, dist, normal, shape = QueryRaycast(point.pos, extend_dir, 2 * ff.resolution, 0.025)
        if hit then 
            -- log the contact, don't create a new extension
            local hit_point = VecAdd(point.pos, VecScale(extend_dir, dist))
            table.insert(ff.contacts, inst_field_contact(point, hit_point, normal, shape))
        else 
            -- extend into the new space at full force
            extension_point = inst_field_point(extend_coord, ff.resolution)
            set_point_vec(extension_point, point.vec)
            -- commented for now since this seems to be working fine without decaying
            -- the parent for every extension, but if I decide to do this again it will
            -- require a patch to the existing setting of all current subscribers... and 
            -- I'm not prepared to do that to them just yet. 
            -- set_point_vec(point, VecScale(point.vec, (1 - ff.prop_decay))) 

            -- since we created this point it's now an edge of the field. It will
            -- be eligible for extension on the next cycle.
            extension_point.edge = true
            field_put(ff.field, extension_point, extension_point.coord)
        end
    end
end

function propagate_field_forces(ff)
    -- propagate the force outside of each point into the coord its pointing at 
    -- and average the vectors, reducing the parent mag by a proportion. 
    local points = flatten(ff.field)
    for i = 1, #points do
        local point = points[i]
        -- propagate the force in the direction of the parent vector
        propagate_point_force(ff, point)
        -- propagate the force in a spread to other vectors around the direction it's pointing.
        -- See extension method above for details about radiate(). Note: without damping, this 
        -- would create a sustaining or increasing energy situation, so this is not physically accurate
        -- but works well for simulation with proper tuning. 
        local prop_dirs = radiate(point.vec, ff.extend_spread, ff.point_split)
        for i = 1, #prop_dirs do
            -- propagate the force in the direction of radiation spokes
            local prop_dir = prop_dirs[i]
            local temp_point = inst_field_point(point.coord, ff.resolution)
            set_point_vec(temp_point, VecScale(prop_dir, point.mag))
            propagate_point_force(ff, temp_point)
        end
    end
end

function propagate_point_force(ff, point)
    -- propagate force to a vector in a target coordinate given a parent vector.
    local force_dir = point.dir
    local coord_prime = round_vec(VecAdd(point.coord, force_dir))
    local point_prime = field_get(ff.field, coord_prime)
    if point_prime ~= nil then 
        -- direction of combined vector at target point is a normalization
        -- of the combined dirs (to unit vector) and average of the magnitudes.
        local dir = VecAdd(point_prime.vec, point.vec)
        dir = VecAdd(dir, Vec(0, ff.heat_rise, 0))
        dir = VecNormalize(dir)
        local mag = (point_prime.mag + point.mag) / 2
        -- set the target point force vector and reduce the parent by
        -- the proportional decay.
        set_point_vec(point, VecScale(point.vec, (1 - ff.prop_decay))) 
        set_point_dir_mag(point_prime, dir, mag)
    end
end

function normalize_field(ff)
    -- Remove points above the set limit of points to simulate in the field. 
    -- Go through all points, clamp the field vector magnitudes to the max
    -- and cull the field vectors that fall below dead magnitude. 
    -- Reduce all magnitudes by a universal proportion. 

    -- remove points until we're under the sim limit
    local points = flatten(ff.field)
    if #points > FF.MAX_SIM_POINTS then
        -- sort the points from least to greatest magnitude
        while #points > FF.MAX_SIM_POINTS do
            local index = math.random(#points) --math.ceil((1.6 * math.random() - 1)^2 * #points)
            if index ~= 0 then 
                local remove_point = points[index]
                field_put(ff.field, nil, remove_point.coord)
                table.remove(points, index)
            end
        end
    end

    -- Apply a universal decay to all points, reducing their magnitudes and 
    -- culling points that fall below dead magnitude. If any point magnitude 
    -- is above max then it will be clamped to the maximum magnitude allowed.
    local points = flatten(ff.field)
    for i = 1, #points do
        local point = points[i]
        set_point_vec(point, VecScale(point.vec, (1 - ff.decay)))
        if point.mag > ff.f_max then 
            set_point_vec(point, VecScale(point.dir, ff.f_max))
        elseif point.mag < ff.f_dead then 
            field_put(ff.field, nil, point.coord)
        end
    end
end

function refresh_metafield(ff)
    -- Rebuild the metafield by averaging the points of the base field in 
    -- the coordinates of the lower resolution metafield. The result is a 
    -- metafield that is smaller and summarizes what's gpoing on in larger 
    -- blocks.
    local new_metafield = {}
    local points = flatten(ff.field)
    for i = 1, #points do
        local point = points[i]
        -- find the meta-coordinate for this base field point. It will belong 
        -- to the same meta-coord as several other base field points and be 
        -- combined (if the resolution of the metafield is lower)
        local meta_coord = Vec(
            math.floor(point.pos[1] / ff.meta_resolution),
            math.floor(point.pos[2] / ff.meta_resolution),
            math.floor(point.pos[3] / ff.meta_resolution))
        local meta_point = field_get(new_metafield, meta_coord)
        -- first base field point in this metafield coordinate, so make a new
        -- point. 
        if meta_point == nil then 
            meta_point = inst_field_point(meta_coord, ff.meta_resolution)
            meta_point.type = point_type.meta
            meta_point.vec = point.vec
            field_put(new_metafield, meta_point, meta_point.coord)
        else
            -- Average the base field point into the existing metafield point.
            set_point_vec(meta_point, VecScale(VecAdd(meta_point.vec, point.vec), 0.5))
        end
    end

    ff.metafield = new_metafield
end

function pos_to_coord(pos, resolution)
    return Vec(
        math.floor(pos[1] / resolution),
        math.floor(pos[2] / resolution),
        math.floor(pos[3] / resolution))
end

function field_put(field, value, coord)
    -- Puts a value into a field at a coordinate.
    -- Fields are a hashed multidim array. They automatically allocate when 
    -- needed and will automatically deallocate when elements are set to nil.
    -- Optimized for fast access.

    -- field["points"] is a cache of the flattened
    -- multidimensional array. Clearing it forces
    -- regeneration. This is cleared whenever the field changes. 

    local xk = tostring(coord[1])
    local yk = tostring(coord[2])
    local zk = tostring(coord[3])

    -- allocate
    if field[xk] == nil then
        field[xk] = {}
    end

    if field[xk][yk] == nil then
        field[xk][yk] = {}
    end

    if field[xk][yk][zk] == nil then 
        field["points"] = nil
    end

    -- set
    field[xk][yk][zk] = value

    -- deallocate
    if value == nil then 
        field["points"] = nil
        local count = pairs(field[xk][yk])
        if count == 0 then 
            field[xk][yk] = nil
        end

        count = pairs(field[xk])
        if count == 0 then 
            field[xk] = nil
        end
    end
end

function field_get(field, coord)
    -- Get a value from a field coordinate.
    -- See field_put() for description of what a field is and how it
    -- operates.
    local xk = tostring(coord[1])
    local yk = tostring(coord[2])
    local zk = tostring(coord[3])
    if field[xk] == nil then
        return nil
    end
    if field[xk][yk] == nil then
        return nil
    end
    if field[xk][yk][zk] == nil then
        return nil
    end
    return field[xk][yk][zk]
end

function flatten(field)
    -- flatten the entire field into a list of points (values at coordinates).
    -- Will return a cached list unless that list is nil.
    if field["points"] == nil then 
        local points = {}
        for x, yt in pairs(field) do
            for y, zt in pairs(yt) do
                for z, point in pairs (zt) do
                    table.insert(points, point)
                end
            end
        end
        field["points"] = points
    end

    return shallow_copy(field["points"])
end

function debug_field(ff)
    -- debug the field by showing vector line indicators
    local points = flatten(ff)
    for i = 1, #points do
        local point = points[i]
        local color = debug_color(ff, point)
        DebugCross(point.pos, color[1], color[2], color[3])
        DebugLine(point.pos, VecAdd(point.pos, point.vec), color[1], color[2], color[3])
    end
end

function debug_color(ff, point)
    -- color code the debug vector line by the proportion 
    -- of maximum force it is. 
    local r = point.mag / ff.f_max
    local b = 1 - r
    return Vec(r, 0, b)
end

function force_field_ff_tick(ff, dt)
    -- debug_field(ff.field)

    if ff.tick_count == 0 then ff.tick_count = ff.tick_reset end
    if (ff.tick_count + 1) % 2 == 0 then 
        -- Do these cycles on every other tick
        propagate_field_forces(ff)
        normalize_field(ff)
        refresh_metafield(ff)
    elseif ff.tick_count % 2 == 0 then 
        -- Extend the field on its own cycle every other tick
        extend_field(ff)
    end
    ff.tick_count = ff.tick_count - 1
end

point_type = enum {
	"base",
	"meta"
}
