from gym.envs.registration import register

register(
    id='fwmav_hover-v0',
    entry_point='flappy.envs.fwmav:FWMAVSimEnv',
)

register(
    id='fwmav_hover-v1',
    entry_point='flappy.envs.fwmav:FWMAVSimEnvSimple',
)

register(
    id='fwmav_maneuver-v0',
    entry_point='flappy.envs.fwmav:FWMAVManeuverEnv',
)
