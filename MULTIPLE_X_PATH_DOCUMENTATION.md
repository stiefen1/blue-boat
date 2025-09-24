# Multiple X-Path Functionality for plot_gnc_data_multi

## Overview

The `plot_gnc_data_multi` method now supports multiple x-coordinate specifications, allowing different series to use different x-axes on the same plot. This enables more flexible visualization of multi-dimensional data and trajectory comparisons.

## New x_path Parameter Options

The `x_path` parameter can now be:

1. **None** (default): Use time as x-axis for all series
2. **String**: Single x_path used for all series (existing behavior)
3. **List[str]**: Different x_path for each series (new functionality)
4. **List with None values**: Mix of time and custom coordinates (new functionality)

## Examples

### 1. Single X-Path (Existing Behavior)
```python
# Use same x-coordinate for all series
sim.plot_gnc_data_multi(
    data_paths=["navigation.eta[1]", "vessel.eta[1]"],
    x_path="navigation.eta[0]",  # North position for both
    labels=["Navigation East", "Vessel East"]
)
```

### 2. Multiple X-Paths (New Functionality)
```python
# Different x-coordinate for each series
sim.plot_gnc_data_multi(
    data_paths=["navigation.eta[1]", "vessel.eta[1]"],
    x_path=["navigation.eta[0]", "vessel.eta[0]"],  # Different north coordinates
    labels=["Navigation Trajectory", "Vessel Trajectory"]
)
```

### 3. Mixed Coordinates (New Functionality)
```python
# Mix time-based and position-based plotting
sim.plot_gnc_data_multi(
    data_paths=["vessel.eta[0]", "vessel.eta[1]"],
    x_path=[None, "vessel.eta[0]"],  # Time for first, north for second
    labels=["North vs Time", "East vs North"]
)
```

### 4. Complex Multi-Source Comparison
```python
# Compare data from multiple sources with their own coordinates
sim.plot_gnc_data_multi(
    data_paths=[
        "navigation.eta[1]",    # Navigation east
        "guidance.eta_des[1]",  # Guidance desired east
        "vessel.eta[1]"         # Vessel actual east
    ],
    x_path=[
        "navigation.eta[0]",    # vs Navigation north
        "guidance.eta_des[0]",  # vs Guidance desired north
        "vessel.eta[0]"         # vs Vessel actual north
    ],
    labels=["Navigation", "Guidance", "Vessel Actual"],
    title="Multi-Source East-North Comparison"
)
```

## Use Cases

### 1. Trajectory Comparison
Compare actual vs estimated trajectories where each has its own coordinate system:
- Navigation system estimated trajectory
- Vessel actual trajectory
- Guidance system desired trajectory

### 2. Mixed Time-Series and Spatial Data
Plot some series against time and others against position on the same graph:
- North position vs time
- East position vs north position (trajectory shape)
- Velocity vs time

### 3. Multi-Source Data Analysis
Analyze data from different sources that may have slightly different coordinate systems:
- Different navigation filters
- Multiple sensor systems
- Various control algorithms

## Error Handling

The method validates that:
- If `x_path` is a list, its length must match the number of `data_paths`
- Each x_path string must be valid and extractable
- Fallback to time if x_path extraction fails

```python
# This will raise ValueError
sim.plot_gnc_data_multi(
    data_paths=["vessel.eta[0]", "vessel.eta[1]"],
    x_path=["vessel.eta[0]"]  # Only one x_path for two data_paths - ERROR!
)
```

## Benefits

1. **Flexible Visualization**: Plot different types of data relationships on the same graph
2. **Trajectory Comparison**: Compare trajectories from different sources with their own coordinate systems
3. **Multi-Dimensional Analysis**: Visualize complex relationships between different coordinate systems
4. **Backward Compatibility**: All existing code continues to work unchanged

## Technical Implementation

- Uses the existing `_extract_data_from_path` helper method for consistency
- Maintains the same data extraction and validation logic
- Supports all existing path notation (e.g., "vessel.eta[0]", "actuators[1].tau[2]")
- Proper error handling with fallback to time-based plotting
- Type annotations for better IDE support

## Type Signature

```python
def plot_gnc_data_multi(
    self, 
    data_paths: List[str], 
    labels: List[str] = None,
    colors: List[str] = None, 
    linestyles: List[str] = None,
    fig_size: Tuple[int, int] = (12, 8), 
    title: str = None,
    ylabel: str = None, 
    xlabel: str = None,
    x_path: Union[str, List[Union[str, None]], None] = None
) -> plt.Figure
```