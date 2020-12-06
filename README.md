# Basics-mobile-robotics
## Set-up

### Generate SSH
    
    ssh-keygen
    
    ssh-add
    
    cat $HOME\.ssh\id_rsa.pub
    
copy stuff and paste it in Github -> Settings -> SSH and GPG keys -> New

if problem check: https://medium.com/rkttu/set-up-ssh-key-and-git-integration-in-windows-10-native-way-c9b94952dd2c

### git

install git on windows: https://git-scm.com/book/en/v2/Getting-Started-Installing-Git/

keep standard settings

if problems check: https://stackoverflow.com/questions/4492979/git-is-not-recognized-as-an-internal-or-external-command

## PyCharm

install plugin in Settings -> Plugins

- github, environment file    (mendatory)
- material theme UI              (nice theme Material Oceanic)

## Github

https://github.com/KookaS/basics-mobile-robotics

### Init

in the folder you want to work on:

    git init
    
    git pull git@github.com:KookaS/basics-mobile-robotics.git
    
### Libraries

Anaconda is useful for setting up python, you have to set as interpreter for the project

packages to install:
    
    conda install ...
    
If you don't have Anaconda download pip3 and then to install libraries:

    pip3 install ...
    
Here is the list of packages to install:

    pyserial python-dotenv opencv-python tqdm matplotlib numpy ipywidgets
    
on linux:

    pip install -U python-dotenv
    
    https://linuxize.com/post/how-to-install-opencv-on-ubuntu-18-04/
    
### Branches

You need everytime you code that you are in an other branch than the main one, especially if you commit and push your code.

I would suggest that before creating a new branch you pull the recent changes:

    git checkout master
    
    git pull

To create a new branch from your repo:

    git checkout -b <your-branch-name>
    
### Pushing code

When you are in your branch, check first if there are conflicts with main branch:

Pycharm:

    click on the icon top right update project

Terminal:

    git checkout master
    
    git pull
    
    git checkout <your-branch-name>
    
    git merge master
    
Now that your are sure that the code still runs well you need to commit and push your code:

Pycharm: 

    click the icon on the top right after reboot of Pycharm: commit
    
        enter your commit message, explain what you did
        
    click the icon push and push your branch to github
    
if you don't see your branch pushed and it is your first attempt click again on push:

    you'll see <your-branch-name> -> master
    
click on master and define what it is(origin/master) with the url of the project. Push again and then link Pycharm to Github
    
Terminal:

    git commit -m "your message to explain here"
    
    git push .....      you will have some comments like push set-upstream, just do what they say
  
Now you need to merge your branch on github. YOu go to the page of the project and click create PR of you branch to the main one.
    
## .env

Create a .env and make sure not to commit and push it

This is a way to store locally variables that can be used in the project, variables that may be different for everyone

    COM_PORT=\\.\COM10
    LEFT_WHEEL_SCALING=98
    RIGHT_WHEEL_SCALING=100
    HALF_TURN_TIME=4.72
    DISTANCE_TIME=0.32
    CAMERA_PORT=1
    SPEED_80_TO_MM_S=0.3333

from terminal:

    cat <<'EOF' > .env
    COM_PORT=\\.\COM10 \
    LEFT_WHEEL_SCALING=98 \
    RIGHT_WHEEL_SCALING=100 \
    HALF_TURN_TIME=4.72 \
    DISTANCE_TIME=0.32 \
    CAMERA_PORT=1 \
    SPEED_80_TO_MM_S=0.3333 \
    EOF
        
## documentation

For every function make sure to document everything, reStructured Text:

    def your_function(arg1: bool, arg2: string):
        """
        Here the description of your function. The description below should tell for each argument and/or return the type and its description.
        If the type is already given above you don't have to write it below.

        :type arg1:     bool
        :param arg1:    arg1 is ...
        :type arg2:     string
        :param arg2:    arg2 is ...
        
        :rtype: list
        :returns: a list of ...
        """ 
        
        your code here...
        return ...

