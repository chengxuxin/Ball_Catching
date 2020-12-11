import pybullet as p


def create_elastic_ball(ball_x, ball_y, ball_z):
    '''
    Create an elastic ball in the simulation.
    Returns the created ball's id.
    '''
    ball_initial_position = [ball_x, ball_y, ball_z]
    ball_visual_id = p.createVisualShape(shapeType=p.GEOM_SPHERE,
                                         radius=0.1,
                                         rgbaColor=[0.75, 0.3, 0, 1],
                                         specularColor=[0.4, .4, 0])

    ball_collision_id = p.createCollisionShape(shapeType=p.GEOM_SPHERE,
                                               radius=0.1)

    # create the ball with visual, collision, and initial position
    ball_id = p.createMultiBody(baseMass=0.5,
                                baseCollisionShapeIndex=ball_collision_id,
                                baseVisualShapeIndex=ball_visual_id,
                                basePosition=ball_initial_position)
    p.changeDynamics(ball_id, -1,
                     lateralFriction=0,
                     spinningFriction=0,
                     rollingFriction=0,
                     restitution=0.8)
    return ball_id
