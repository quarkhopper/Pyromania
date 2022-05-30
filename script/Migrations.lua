#include "Utils.lua"

function migrate_option_set(option_set_ser)
    local version = 0
    local parts = split_string(option_set_ser, DELIM.OPTION_SET)
    version = parts[3]

    if version == "1" then
    	-- from version 1.0 to 1.1 with min max random explode distance
        version = "1.1"
        local set_parts = split_string(option_set_ser, DELIM.OPTION_SET)
        set_parts[3] = version
        local option_name = set_parts[1]
        if option_name == "bomb" then 
            local min_random_radius = create_mode_option(
                option_type.numeric, 
                15,
                "min_random_radius",
                "Minimum radius of random explosions")
            min_random_radius.range.lower = 0
            min_random_radius.range.upper = 100
            min_random_radius.step = 1
            local ser = mode_option_to_string(min_random_radius)
            set_parts[#set_parts + 1] = ser
            
            local max_random_radius = create_mode_option(
                option_type.numeric, 
                30,
                "max_random_radius",
                "Maximum radius of random explosions")
            max_random_radius.range.lower = 0
            max_random_radius.range.upper = 100
            max_random_radius.step = 1
            local ser = mode_option_to_string(max_random_radius)
            set_parts[#set_parts + 1] = ser
        end
        option_set_ser = join_strings(set_parts, DELIM.OPTION_SET)
    end

    if version == "1.1" then
    	-- from version 1.1 to 1.2 with propagation decay
        version = "1.2"
        local set_parts = split_string(option_set_ser, DELIM.OPTION_SET)
        set_parts[3] = version
        local new_parts = {set_parts[1], set_parts[2], set_parts[3]}
        for i = 4, #set_parts do
            local option_ser = set_parts[i]
            local option = mode_option_from_string(option_ser)
            if option.key == "decay" then
                -- update the step amount of the decay option and change the friendly name
                option.friendly_name = "Whole field normalization decay per tick"
                option.step = 0.001
                option = mode_option_to_string(option)
                new_parts[#new_parts + 1] = option

                -- put the new option after the decay option
                local prop_decay = create_mode_option(
                    option_type.numeric, 
                    0.01,
                    "prop_decay",
                    "Field propagation decay per tick")
                prop_decay.range.lower = 0
                prop_decay.range.upper = 0.3
                prop_decay.step = 0.001
                prop_decay = mode_option_to_string(prop_decay)
                new_parts[#new_parts + 1] = prop_decay
            else
                new_parts[#new_parts + 1] = option_ser
            end
            option_set_ser = join_strings(new_parts, DELIM.OPTION_SET)
        end
    end

    return option_set_ser
end
