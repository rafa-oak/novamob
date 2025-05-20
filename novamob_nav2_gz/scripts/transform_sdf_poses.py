import re

# Transformation values
dx = 0.73164
dy = 1.469743

# Regex pattern to match <pose> tags with 6 numeric values
pose_pattern = re.compile(r"<pose>([-\d.eE+]+) ([-\d.eE+]+) ([-\d.eE+]+) ([-\d.eE+]+) ([-\d.eE+]+) ([-\d.eE+]+)</pose>")

def transform_pose_line(line):
    match = pose_pattern.search(line)
    if match:
        x, y, z, roll, pitch, yaw = map(float, match.groups())
        if x != 0.0 or y != 0.0:  # Only transform non-zero x or y
            x_new = x + dx
            y_new = y + dy
            # Format with same number of decimal places as input
            new_pose = f"<pose>{x_new:.5f} {y_new:.5f} {z} {roll} {pitch} {yaw}</pose>"
            return pose_pattern.sub(new_pose, line)
    return line

def process_sdf_file(input_path, output_path):
    with open(input_path, 'r') as infile, open(output_path, 'w') as outfile:
        for line in infile:
            outfile.write(transform_pose_line(line))

# Example usage
if __name__ == "__main__":
    input_sdf = "/home/rafael/ros2_ws/src/novamob_nav2_gz/world/ign_indoor/ign_indoor_centered.sdf"
    output_sdf = "transformed_world.sdf"
    process_sdf_file(input_sdf, output_sdf)
    print(f"Transformed SDF saved to '{output_sdf}'")
