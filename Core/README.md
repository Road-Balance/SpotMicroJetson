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


```
# activate python venv
source <your-env>/bin/activate

# run automatic gait
# this needs administer privileges for keyboard multiprocessing
sudo python3 example_automatic_gait.py
```

