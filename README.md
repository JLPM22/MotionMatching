# Motion Matching

![](.github/media/path_mm.jpg)

------

📚 **[Documentation](https://jlpm22.github.io/motionmatching-docs/)**

Welcome to the **Motion Matching** implementation designed for the **Unity** game engine. This project originated from the author's master thesis, providing a deep dive into both the Motion Matching technique and the workings of this specific Unity package. Download the complete thesis [here](https://www.researchgate.net/publication/363377742_Motion_Matching_for_Character_Animation_and_Virtual_Reality_Avatars_in_Unity) for an extensive overview. The project is a work-in-progress, aiming to offer a comprehensive Motion Matching solution for Unity. It can serve as a useful resource for those keen to learn or implement their own Motion Matching solution or even extend this existing package.

![](.github/media/architecture_diagram.PNG)

# Quick Start Guide

Follow these steps to get started with the Motion Matching package for Unity. Visit the 📚 **[Documentation](https://jlpm22.github.io/motionmatching-docs/)** for an in-depth description of the project.

## Installation Steps

1. Ensure you have **Unity 2021.2+** installed (untested on other versions).

2. Open the Unity Editor and navigate to **Window > Package Manager**.

3. In the Package Manager, click **Add (+) > Add package by git URL...**.

4. Insert the following URL into the git URL field and click **Add**:
	```
	https://github.com/JLPM22/MotionMatching.git?path=/com.jlpm.motionmatching
	```

	> Note: All sample scenes use the Universal Render Pipeline (URP). Conversion may be necessary if you are using a different render pipeline.

5. *[Optional]* In the Package Manager, click on **Motion Matching**, then import the example scenes by selecting **Samples > Examples > Import**.

6. *[Optional]* Go to ``Examples/Scenes/JLTest`` in the Project Window to explore the sample scenes.

## Project Overview

### Directories

- `Samples/Animations`: Contains motion capture (MoCap) files (with *.txt* extensions but originally *.bvh* files) and *MMData* files to define the animation database for the Motion Matching System.
  
- `StreamingAssets/MMDatabases`: Contains the processed pose and feature databases, as well as skeletal information. This directory is automatically created when generating databases from an *MMData* file.

### Key Components

Demo scenes consist of two primary GameObjects:

1. **Character Controller**: Creates trajectories and imposes positional constraints, like limiting the maximum distance between the simulated and animated character positions.

2. **MotionMatchingController**: Handles all Motion Matching operations. It provides adjustable parameters for enabling/disabling features like inertialize blending or foot locking.

Feel free to tweak and explore these components to get a better understanding of the system.

# Roadmap

Here's a list of upcoming features and improvements to enhance the capabilities and usability of the Motion Matching package for Unity:

## Planned Features

- [ ] **Enhanced Documentation**: Work on a more comprehensive guide explaining the intricacies of Motion Matching. For now, you can refer to the author's [master thesis](https://www.researchgate.net/publication/363377742_Motion_Matching_for_Character_Animation_and_Virtual_Reality_Avatars_in_Unity).

- [ ] **Trajectory and Pose Feature Customization**: Implement a more user-friendly system for modifying trajectory features, and expand the types of pose features available.

- [ ] **Motion Capture Library**: Record additional motion capture data and create default avatars for immediate use in projects.

- [ ] **PathCharacterController Upgrades**: Revise the current hardcoded path tools in favor of a more flexible system, possibly using splines or other advanced techniques.

- [ ] **Visual Debugging Tools**: Develop a suite of visual debugging tools for easier inspection and adjustment during development.

- [ ] **Memory Optimization with Deep Learning**: Investigate the use of deep learning algorithms to reduce memory usage in the system.

## Completed Features

- [x] **BVH Editor with Tagging**: Introduced a BVH editor that supports real-time complex tag queries.
  
- [x] **Unity Animation System Integration**: Added tools and methods for seamless integration with Unity's native animation system. This allows, for example, the use of Unity's animation system for the upper body while utilizing Motion Matching for the lower body.

- [x] **Unity Package Structure**: Successfully restructured the project into a Unity package for easier distribution and integration.

Your contributions and suggestions are always welcome as we continue to develop this project into a comprehensive Motion Matching solution for Unity.

## Citation

If you find this package beneficial, kindly attribute it to this repository or cite the author's master thesis using the following citation:

```plaintext
@mastersthesis{ponton2022mm,
  author  = {Ponton, Jose Luis},
  title   = {Motion Matching for Character Animation and Virtual Reality Avatars in Unity},
  school  = {Universitat Politecnica de Catalunya},
  year    = {2022},
  doi     = {10.13140/RG.2.2.31741.23528/1}
}
```

## License

This project is distributed under the MIT License. For complete license details, refer to the [LICENSE](https://github.com/JLPM22/MotionMatching/blob/main/LICENSE) file.
