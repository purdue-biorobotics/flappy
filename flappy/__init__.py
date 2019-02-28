from gym.envs.registration import register

register(
    id='fwmav-v0',
    entry_point='flappy.envs:FWMAVSimEnv',
)
