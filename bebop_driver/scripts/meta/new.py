import re 

def convert_to_snake_case(s):
    # Replace each uppercase letter with an underscore followed by the lowercase equivalent
    snake_case = re.sub(r'([A-Z])', r'_\1', s).lower()
    # If the string starts with an underscore, remove it
    if snake_case.startswith('_'):
        snake_case = snake_case[1:]
    return snake_case

# Example usage:
input_string1 = "ThisIsAnExampleString"
output_string1 = convert_to_snake_case(input_string1)
print(output_string1)  # Output: "this_is_an_example_string"

input_string2 = "swVersion"
output_string2 = convert_to_snake_case(input_string2)
print(output_string2)  # Output: "sw_version"

input_string2 = "sw_version"
output_string2 = convert_to_snake_case(input_string2)
print(output_string2)  # Output: "sw_version"

input_string3 = "zAxisCalibration"
output_string3 = convert_to_snake_case(input_string3)
print(output_string3)  # Output: "z_axis_calibration"

# Original string with .\n characters
original_string = "Hello.\nThis is a string.\nwith new line characters."

# Remove .\n characters
cleaned_string = original_string.replace('.\n', '.')

print(cleaned_string)