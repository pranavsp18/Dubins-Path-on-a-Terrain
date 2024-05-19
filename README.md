# Computing Dubins' Path on a Terrain

## Problem statement
This problem is a part of Mechanical Engineering coursework at Texas A&M University (MEEN 689 - SPTP: Decision making Algorithms for UAVs). It computes a shortest path from any given point on the terrain to any other point on the terrain for a curvature constrained vehicle (Dubins' Vehicle/ Fixed-wing UAV).

The terrain is modelled as a piece-wise linear function:

               `z = max{0, max{cix+diy+ei}}  i = 1,2,...,N`

- For simplicity of the problem, the terrain is assumed to be

               `z = max{0, x-y,2x+3y+1}`

- **Turning circle radius** - Unit dimension
- **Vehicle speed** - Unit dimension

## Getting Started

### Working

Each plane is rotated about its intersection with *z = 0* plane and a new point in the rotated plane is obtained. This is done for both initial and final points. Dubins' path is obtained with these new points along with heading at each location.

### Script

- `Dubins_path_on_a_terrain.jl`: Contains code which computes the Dubins' path.

### Usage

- Step 1: Clone the github repository to your local drive using the link 

```bash
git clone https://github.com/kaarthiksundar/Dubins.jl.git
```

- Step 2: Navigate to `src` folder in the `Dubins.jl` repository on your workstation.

- Step 3: Clone this repository into `Dubins.jl/src` folder using.

```bash
git clone https://github.com/kaarthiksundar/Dubins.jl.git
```

Step 3: Run `Dubins_path_on_a_terrain.jl` to compute Dubins shortest path.
