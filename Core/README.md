## SpotMicroAI Pybullet Simulation

![auto_gait](https://user-images.githubusercontent.com/12381733/95225571-e64d1680-0836-11eb-9077-2ad557a4aaf2.gif)


## Install Dependencies

```
sudo apt-get install python3 python3-pip
git clone https://github.com/Road-Balance/SpotMicroJetson.git

source <your-env>/bin/activate

cd SpotMicroJetson/Core
pip3 install -U -r requirements.txt 
```

## Automatic Gait with Keyboard Control

![keyboard_control](https://user-images.githubusercontent.com/12381733/95225220-8191bc00-0836-11eb-8a31-23583954a9d5.gif)


Control SpotMicro in Pybullet Simulation with keyboard buttons 


```
# activate python venv
source <your-env>/bin/activate

# run automatic gait
# this needs administer privileges for keyboard multiprocessing
sudo python3 pybullet_automatic_gait.py

# if you're using venv
sudo <your-env-python-location> pybullet_automatic_gait.py
```

* `w` : go ahead
* `a` : go leftward
* `s` : go backward
* `d` : go rightward
* `q` : turn counter-clockwise
* `e` : turn clockwise
* `space` : Stop

Each time you press a button, it will move harder in that direction.


