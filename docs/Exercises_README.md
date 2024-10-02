# Duckietown Exercises

REQUIRED READING: [docs/vnc-docker_README.md](./vnc-docker_README.md)

The exercises are run inside the `main-workspace` container. However, the actual exercise repositories will be stored outside the docker container, in the directory [`user_code_mount_dir/`](../user_code_mount_dir) of this `vnc-docker` repository. This directory is mounted inside the docker container at `/code/catkin_ws/src/user_code/`. This means that whatever changes you make within [`user_code_mount_dir/`](../user_code_mount_dir) outside the docker container, it will be instantly reflected within the folder `/code/catkin_ws/src/user_code/` inside the docker container, and vice-versa.

##  Clone the exercise repositories
You first need to setup your GitHub account on the Duckiebot, and being a member of the GitHub [`ETHZ-DT-Class`](https://github.com/ETHZ-DT-Class) organization.

Then, clone the required exercise repository inside the [`user_code_mount_dir/`](../user_code_mount_dir) folder of this `vnc-docker` repository. **It is highly suggested to clone the repositories *outside* the docker container within the mentioned folder, and not directly inside the container, to avoid permission issues**.

You may want to [setup a SSH key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent) on the Duckiebot to avoid typing your GitHub credentials every time you clone a repository.

```
git clone git@github.com:ETHZ-DT-Class/exerciseX.git  /home/duckie/vnc-docker/user_code_mount_dir/exerciseX
```

##  Updates
During the class, an exercise could be updated with bug fixes. If this is the case, we will ask you to update the git repository of the exercise and to work with the new version of the exercise only. To do so, you have to `git pull` while being inside the specific exercise folder. Then you may run `catkin build` inside the docker container to build the updated exercise package.

You ***DO NOT*** need to stop and remove the current running container when updating the exercise repository, or to pull/build a new version of the docker image. You can continue to work with the same running container. You just have to update the git repository of the exercise.

We will update on Moodle the version number of the exercises you should develop, to help you confirm that you have correctly update them. To check this, simply run `make show-versioning` *outside* the container, or manually check the *version* tag inside the `package.xml` file of the exercise repository.

**IMPORTANT** ROS packages have a version with the format *X.Y.Z* ([semantic versioning](https://semver.org/)). For this class, the version will only change in its X and Y values, with the following meaning:
- increase in X value (major update): we updated the code in some way, or anyway there is an important update, you **must** update the package. 
- increase in Y value (minor upddate): we updated the instructions/documentation only (comments in code, or READMEs, ...), you don't have to update the package (but is highly recommended)

##  Complete the exercises
Each exercise repository contains a `README.md` file with the instructions to complete the exercise and evaluate it on local public test cases.

Remember that since the exercise repositories are ROS packages, before running an exercise you need to run (only once) `catkin build exerciseX` at whatever subfolder level inside `/code/catkin_ws/` to build the exercise package, or run `catkin build --this` if you are already inside the exercise folder. Then source the workspace with `source /code/catkin_ws/devel/setup.bash`.

###  Evaluation reports
The evaluation pipeline of each exercise will create reports in both HTML and PDF format for you to review, located at `exerciseX/reports/`.

You have multiple options to visualize the reports:

- open the reports on the Duckiebot VNC desktop environment with the already installed `Firefox Web Browser` application. 
**Click here if you cannot visualize the video below: [assets/media/display-reports.mp4](./assets/media/display-reports.mp4)**.
![](./assets/media/display-reports.mp4)

- if you are connected to the Duckiebot with VS Code SSH, you can install the extensions [`Live Server`](https://marketplace.visualstudio.com/items?itemName=ritwickdey.LiveServer) and [`vscode-pdf`](https://marketplace.visualstudio.com/items?itemName=tomoki1207.pdf) to visualize the reports directly in VS Code.

- you can copy the reports to your laptop and visualize them locally there. Run on your laptop:
```
scp -r duckie@<DUCKIEBOT_NAME>.local:/<PATH_TO_VNC-DOCKER_ON_DUCKIEBOT>/user_code_mount_dir/exerciseX/reports <WHATEVER_PATH_ON_LAPTOP>/exerciseX_reports
```

##  Submit the exercises
Check Moodle and the `README.md` file of the exercise repository for the submission instructions.