# Non-linear predictive controller for Gantry Crane

This repo contains a matlab implementation of a non-linear predictive controller for a gantry crane. 

The goal is to move the gantry crane from point A to point B without touching any of the obstacles (see fig. here below), while minimizing the energy used by the crane's motors. 

![Example Test Course](https://github.com/kubitz/PredictiveControlGantry/blob/master/figures/diagram.png)


## Repo Structure
- All the obstacles and course simensions are specificied in `DefaultCourse.m` file. 
- All of the functions are in `FunctionTemplate.m` 
- To start the controller, run `TestMyDesign_NonLinear.m`

A general overview of the design choices made while designing is available in the `Report` folder. 

