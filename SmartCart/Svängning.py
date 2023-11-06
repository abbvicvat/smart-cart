import math

# Example usage:
position = (0, 0)  # Current position of the vehicle (x, y)
target = (0, 0.01)  # Position of the target (x, y)
max_speed = 2.3  # Maximum speed of the vehicle

def calculate_speed(position, target, max_speed):
    """Calculate the speed of the wheels based on the target position."""
    # Calculate the distance to the target
    dx = target[0] - position[0]
    dy = target[1] - position[1]
    distance = (dx**2 + dy**2)**0.5

    # Calculate the time to reach the target
    time_to_reach_target = distance / max_speed

    # If the target is very close or too far, stop the vehicle
    if distance < 0.5 or distance > 12:
        return 0, 0

    # If the vehicle is already at the same y-coordinate as the target, move straight
    if dy == 0:
        left_speed = max_speed
        right_speed = max_speed
    else:
        # Calculate the angle to the target
        angle = math.atan2(dy, dx)

        # Calculate the speed of each wheel based on time (inversely proportional to time)
        left_speed = max_speed * (1 + angle / math.pi) / time_to_reach_target
        right_speed = max_speed * (1 - angle / math.pi) / time_to_reach_target

    return left_speed, right_speed



if position[0] == target[0]:
    # Calculate the distance to the target
    dx = target[0] - position[0]
    dy = target[1] - position[1]
    distance = (dx**2 + dy**2)**0.5

    # Define the maximum distance at which the vehicle should move at max_speed
    max_distance = 1.0  # You can adjust this value as needed

    # Calculate the speed based on the distance to the target
    speed = max_speed * min(distance / max_distance, 1)

    left_speed = speed
    right_speed = speed
else:
    left_speed, right_speed = calculate_speed(position, target, max_speed)


print("Left speed:", left_speed)
print("Right speed:", right_speed)
