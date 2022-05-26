# NerfTurret_prototype
Contains various versions of "Main" for an Automatic Nerf Turret. They are written in Python or C++.

Stud code is used for interfaces to hardware, very little to no code is provided for hardware. These versions only deal with the complex logic and code flow associated with finding a target, determining position and distance, adjusting trajectory, and then firing. Originally, the hardware interfaces were going to be added in a at later date, but subsequent redesigns and hardware problems neccessitated a redesign.

The orginal hardware design was to include
1. horizontal and vertical motors for barrel movement
2. sonic range finder and air pressure sensor for range finder calibration
3. flywheel motors for firing the projectile

The main code flow proceded as follows:

1. Look for target
3. If found, compute horizontal offset
4. Once pointing at the target, compute the range to target
5. With range to target obtained, compute the needed barrel trajectory angle
6. Move turret vertically
7. Fire
8. Repeat 

These code files were later discarded because of hardware limitations and problems. A simple design was used for a "Semi-Automatic" version using face-detection off-loaded to a laptop and using multiple arduinos. See the repository DallinPoole66/NerfTurret
