This repo started out as the codebase that I developed as part of [the paper "Towards computing low-makespan solutions for multi-arm multi-task planning problems"](https://vhartmann.com/robplan-low-makespan/).
Compared to the state in the paper, the stippling is currently not available, but it is much faster, and handovers and other primitives are available.

# Installation
The code depends on [rai](https://github.com/vhartman/rai/tree/changes), [rai-manip](https://github.com/vhartman/rai-manip) and [rai-robotModels](https://github.com/vhartman/rai-robotModels).
Ideally, all repos live in the same folder. Otherwise, some paths have to be adjusted.

For an example of how to build, please have a look at [rai](https://github.com/vhartman/rai/tree/changes), respectively at the [CI](https://github.com/vhartman/24-data-gen/actions).

# Example use
The following command runs a random search on the default (random) environment.
```
./x.exe -mode random_search
```

The follwing command rnus a random search on a user-specified scenario and exports the images, and displays some solutions.
```
./x.exe -pnp true -mode random_search -seed 879 -robot_env 'in/envs/three_opposite_gripper.json' -env 'in/objects/four_obj.json' -display true -export_images true
```

The images that are stored can be made into a video with 
```
ffmpeg -pattern_type glob -framerate 30 -i "*.ppm" -q:v 1 vid.mpeg
```
which would in this case result in the following video:
![Robot Video](./example/example.gif)

#### Flags
There are many flags to specify behaviour. Please refer to `main.cpp` for them.

# Citation
If you use this codebase in your research, please cite the paper where the codebase is from as

```
@misc{23-hartmann-ICAPSws-robplan,
  title = {Towards computing low-makespan solutions for multi-arm
  		  multi-task planning problems},
  author = {Hartmann, Valentin N. and Toussaint, Marc},
  year = {2023},
  howpublished = {ICAPS Workshop on Planning and Robotics},
  arxiv_pdf = {2305.17527}
}
```
