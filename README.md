# random-6dof-trajectory-generator
Attempt to generate random flight trajectories.
Uses Bezier curves to generate a trajectory using control points.
Parameters (hard-coded for now, will add a yaml file shortly):
* generateNRandomPoints: 
    * second argument: number of control points (default: 8)
    * third argument: size of cube in which control points are randomly spawned (default: 20)
* linspace
    * first argument: number of points in trajectory (default: 5000)
