Prepare your thirparty repos:
```bash
mkdir src && cd src
vcs import < ../dependencies.repos
git clone --recurse-submodules https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0.git
```