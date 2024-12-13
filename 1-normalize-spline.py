import numpy as np
from typing import List, Tuple

# Input Points
original_points = [
    (-68.61734008789062, 28.412860870361328, -117.1478271484375),
    (-62.09184265136719, 30.651872634887695, -117.21089172363281),
    (-57.91181945800781, 32.483795166015625, -116.83250427246094),
    (-54.225982666015625, 34.69437789916992, -114.7513427734375),
    (-49.748321533203125, 36.67055892944336, -109.45381164550781),
    (-47.3518180847168, 36.67055892944336, -99.61557006835938),
    (-46.7567138671875, 36.670555114746094, -88.02503967285156),
    (-46.99205017089844, 36.670555114746094, -71.42524719238281),
    (-47.64048385620117, 36.67055130004883, -54.397525787353516),
    (-48.234737396240234, 36.67054748535156, -36.86526107788086),
    (-48.35963439941406, 36.67054748535156, -24.945842742919922),
    (-47.47795104980469, 36.67054748535156, -14.287738800048828),
    (-45.748111724853516, 36.6705436706543, -5.752758979797363),
    (-37.00904083251953, 36.6705436706543, 3.4967918395996094),
    (-32.802738189697266, 36.675209045410156, 4.195756912231445),
    (-28.943275451660156, 36.67131042480469, 4.227588653564453)
]

def resample_curve_points(points: List[Tuple[float, float, float]], target_distance: float = 1.0) -> List[Tuple[float, float, float]]:
    """
    Resample points along a curve to have uniform spacing.
    
    Parameters:
    points: List of (x, y, z) coordinates
    target_distance: Desired distance between points (default: 1.0)
    
    Returns:
    List of resampled points with uniform spacing
    """
    if len(points) < 2:
        return points

    # Calculate cumulative distances along the curve
    distances = [0]
    for i in range(1, len(points)):
        p1 = np.array(points[i-1])
        p2 = np.array(points[i])
        distance = np.linalg.norm(p2 - p1)
        distances.append(distances[-1] + distance)
    
    total_length = distances[-1]
    
    # Calculate number of segments needed
    num_segments = int(np.ceil(total_length / target_distance))
    
    # Create new points at uniform distances
    resampled_points = []
    for i in range(num_segments + 1):
        target_distance_along_curve = i * target_distance
        
        # If we've reached the end, append the last point
        if target_distance_along_curve >= total_length:
            resampled_points.append(points[-1])
            break
            
        # Find the segment that contains our target distance
        segment_index = np.searchsorted(distances, target_distance_along_curve) - 1
        segment_index = max(0, min(segment_index, len(points) - 2))
        
        # Calculate how far along the segment our point should be
        segment_start_distance = distances[segment_index]
        segment_length = distances[segment_index + 1] - segment_start_distance
        segment_fraction = (target_distance_along_curve - segment_start_distance) / segment_length
        
        # Interpolate the point
        p1 = np.array(points[segment_index])
        p2 = np.array(points[segment_index + 1])
        new_point = p1 + segment_fraction * (p2 - p1)
        
        resampled_points.append(tuple(new_point))
    
    return resampled_points

def round_to_integers(points: List[Tuple[float, float, float]]) -> List[Tuple[int, int, int]]:
    """
    Round points to nearest integers.
    
    Parameters:
    points: List of (x, y, z) coordinates with floating point values
    
    Returns:
    List of points with integer coordinates
    """
    return [(round(x), round(y), round(z)) for x, y, z in points]

def main():
    # Resample points to have uniform spacing
    resampled_points = resample_curve_points(original_points, target_distance=1.0)
    
    # Round to integers if needed
    integer_points = round_to_integers(resampled_points)
    
    # Print results
    # print("Original points:")
    # for point in original_points:
    #     print(f"  {point}")
        
    # print("\nResampled points (uniform spacing):")
    # for point in resampled_points:
    #     print(f"  {point}")
        
    print("\nInteger points:")
    for point in integer_points:
        print(f"  {point},")

# Usage
if __name__ == "__main__":
    main()
