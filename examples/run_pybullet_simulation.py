import sys
import os

# IMPORTANT: allows us to import from our project's modules directly, 
# even if the script is not run from the project root
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))

import simulation.pybullet.env as pyb_env

def main():
    # create an environment
    # more enviornment names can be found here:
    # https://github.com/utiasSTARS/manipulator-learning/tree/master?tab=readme-ov-file#current-environment-list
    env = pyb_env.create_env("PandaPlayInsertTrayXYZState")
    # env = pyb_env.create_env("ThingLiftXYZState")
    # env = pyb_env.create_env("PandaReachXYZState", render_opengl_gui=False)

    obs = env.reset()
    for acts in range(100):
        next_obs, rew, done, info = env.step(env.action_space.sample())
        env.render()

    env.close()

if __name__ == "__main__":
    main()