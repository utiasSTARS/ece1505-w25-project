import sys
import os

submodule_path = os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', '..', '..', 'external', 'manipulator-learning'))
sys.path.insert(0, submodule_path)

try:
    import manipulator_learning.sim.envs as manipulator_learning_envs
except ImportError as e:
    print(f"Error importing from the submodule at {submodule_path}:")
    print(f"  - Exception: {e}")
    sys.exit(1)

def create_env(env_name, render_opengl_gui=True, egl=False, **kwargs):
    """
    Factory for creating an instance of the specified manipulator-learning environment.

    Args:
        env_name (str): The name of the environment class (e.g., 'PandaPlayXYZState').
        render_opengl_gui (bool): Whether to render using the OpenGL GUI.
        egl (bool): Whether to use EGL for headless rendering.
        **kwargs: Additional keyword arguments passed to the environment constructor.

    Returns:
        An instance of the specified environment.

    Raises:
        AttributeError: If the environment class is not found.
    """
    try:
        env_class = getattr(manipulator_learning_envs, env_name)
    except AttributeError:
        print(f"Error: Environment '{env_name}' not found in manipulator_learning.sim.envs.")
        raise

    return env_class(render_opengl_gui=render_opengl_gui, egl=egl, **kwargs)