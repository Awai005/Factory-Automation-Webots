def compute_bezier_points(control_points, num_points=100):
    """
    Compute points on a Bézier curve given control points.
    
    :param control_points: List of tuples (x, y) representing control points.
    :param num_points: Number of points to compute on the curve.
    :return: List of tuples (x, y) representing points on the Bézier curve.
    """
    n = len(control_points) - 1
    bezier_points = []
    
    def bezier_interp(t):
        """Calculate the point on the Bézier curve for a given t"""
        x = sum(
            binomial_coeff(n, i) * ((1 - t) ** (n - i)) * (t ** i) * p[0]
            for i, p in enumerate(control_points)
        )
        y = sum(
            binomial_coeff(n, i) * ((1 - t) ** (n - i)) * (t ** i) * p[1]
            for i, p in enumerate(control_points)
        )
        return (x, y)
    
    def binomial_coeff(n, k):
        """Calculate the binomial coefficient "n choose k"."""
        from math import factorial
        return factorial(n) / (factorial(k) * factorial(n - k))
    
    for i in range(num_points):
        t = i / float(num_points - 1)
        bezier_points.append(bezier_interp(t))
    
    return bezier_points

# Define your control points based on critical points along your path
control_points = [(6.9, 7.24), (6.9, 6.5), (5.78, 5.72), (4.5, 4.3), (3.6, 3.5), (1.4, 1.3)]

# Generate smooth Bézier curve points
smoothed_path = compute_bezier_points(control_points, num_points=150)

# The smoothed_path can now be used in Webots for navigation purposes
