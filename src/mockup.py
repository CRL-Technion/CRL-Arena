from natnet import RigidBody, Position, Rotation, MarkerSet
from natnet.protocol import MarkerSetType


class ListenerMock:

    def __init__(self, bodies, marker_sets):

        # initialize mock variables for test
        self.bodies = bodies
        self.marker_sets = marker_sets

    def start(self):
        pass

    def stop(self):
        pass

    def update_body(self, new_body, idx):
        # print('RigidBodies {}'.format(bodies))
        self.bodies[idx] = new_body

    def update_marker_set(self, new_marker_set, idx):
        # print('RigidBodies {}'.format(bodies))
        self.marker_sets[idx] = new_marker_set


"""
    Sets of mockup listeners to use for tests.
    Do not change or erase any mockup!
    If you want to run a test with a scenario that doesn't exists in the mockups list, add a new mockup.
    
"""

# Simple listener for init state only, with two agents
simple_listener_mock = ListenerMock(
    bodies=[
        RigidBody(body_id=202, position=Position(x=0.84565269947052, y=-1.5096111297607422, z=0.29082682728767395),
                  rotation=Rotation(w=-0.70233553647995, x=0.01673695631325245, y=-0.7110365033149719, z=-0.02952822856605053)),
        RigidBody(body_id=201, position=Position(x=0.6938132643699646, y=2.6387038230895996, z=0.2628537118434906),
                  rotation=Rotation(w=0.005934544373303652, x=-0.0076073273085057735, y=-0.9919557571411133, z=0.12621665000915527)),
        RigidBody(body_id=102, position=Position(x=-0.8415022492408752, y=-0.39401745796203613, z=0.1720801591873169),
                  rotation=Rotation(w=0.002280650194734335, x=0.004938753787428141, y=-0.04532229155302048, z=0.9989576935768127)),
        RigidBody(body_id=101, position=Position(x=0.34119391441345215, y=1.7233752012252808, z=0.16287793219089508),
                  rotation=Rotation(w=-0.003625936107710004, x=-0.0006945879431441426, y=-0.030593113973736763, z=0.9995251297950745)),
        RigidBody(body_id=305, position=Position(x=-0.6348478198051453, y=-1.105554223060608, z=0.27171966433525085),
                  rotation=Rotation(w=-0.8925718665122986, x=-0.038389548659324646, y=-0.4264281094074249, z=0.1414237916469574)),
        RigidBody(body_id=306, position=Position(x=0.3885113000869751, y=0.6434609293937683, z=0.26068705320358276),
                  rotation=Rotation(w=-0.002968010725453496, x=0.002516773995012045, y=0.6874159574508667, z=-0.7262535691261292))],

    marker_sets=[
        MarkerSet(name="Obstacle1",
                  type=MarkerSetType.Obstacle,
                  positions=[Position(x=0.8615313172340393, y=2.6712381839752197, z=0.24950148165225983),
                             Position(x=0.7881598472595215, y=2.499725103378296, z=0.27377137541770935),
                             Position(x=0.6008504629135132, y=2.7838847637176514, z=0.27146992087364197),
                             Position(x=0.5250619053840637, y=2.599602460861206, z=0.2562762498855591)]),
        MarkerSet(name="Obstacle2",
                  type=MarkerSetType.Obstacle,
                  positions=[Position(x=0.9578632712364197, y=-1.6313328742980957, z=0.29644566774368286),
                             Position(x=0.8070256114006042, y=-1.5637718439102173, z=0.2939743399620056),
                             Position(x=0.9514260292053223, y=-1.360983967781067, z=0.2925402820110321),
                             Position(x=0.7598497271537781, y=-1.638802409172058, z=0.2823731601238251),
                             Position(x=0.7524806261062622, y=-1.3530833721160889, z=0.2888467311859131)]),
        MarkerSet(name="Obstacle3",
                  type=MarkerSetType.Obstacle,
                  positions=[Position(x=-0.645495593547821, y=-0.9943488240242004, z=0.26637980341911316),
                             Position(x=-0.5272716283798218, y=-1.2692276239395142, z=0.29252272844314575),
                             Position(x=-0.5399668216705322, y=-0.9854581952095032, z=0.26634693145751953),
                             Position(x=-0.7380496263504028, y=-0.996252715587616, z=0.26576435565948486),
                             Position(x=-0.7233214974403381, y=-1.2827075719833374, z=0.26748332381248474)]),
        MarkerSet(name="Obstacle4",
                  type=MarkerSetType.Obstacle,
                  positions=[Position(x=0.46581923961639404, y=0.5918686389923096, z=0.26094764471054077),
                             Position(x=0.4655987620353699, y=0.519455075263977, z=0.2603091299533844),
                             Position(x=0.2780880331993103, y=0.5167971253395081, z=0.2596586048603058),
                             Position(x=0.2697637677192688, y=0.7931483387947083, z=0.2662479281425476),
                             Position(x=0.4637969136238098, y=0.7951851487159729, z=0.2565675377845764)]),
        MarkerSet(name="Ruby-1",
                  type=MarkerSetType.Robot,
                  positions=[Position(x=0.3071064054965973, y=1.693137288093567, z=0.1792670041322708),
                             Position(x=0.30719614028930664, y=1.7555233240127563, z=0.15431636571884155),
                             Position(x=0.4096698760986328, y=1.7208905220031738, z=0.15492025017738342)]),
        MarkerSet(name="Rosie-2",
                  type=MarkerSetType.Robot,
                  positions=[Position(x=-0.8268868327140808, y=-0.42351624369621277, z=0.18796001374721527),
                             Position(x=-0.8045069575309753, y=-0.3426115810871124, z=0.16439802944660187),
                             Position(x=-0.8931643962860107, y=-0.4159773290157318, z=0.16380201280117035)]),
        MarkerSet(name="all",
                  type=MarkerSetType.All,
                  positions=[Position(x=0.9578632712364197, y=-1.6313328742980957, z=0.29644566774368286),
                                 Position(x=0.8070256114006042, y=-1.5637718439102173, z=0.2939743399620056),
                                 Position(x=0.9514260292053223, y=-1.360983967781067, z=0.2925402820110321),
                                 Position(x=0.7598497271537781, y=-1.638802409172058, z=0.2823731601238251),
                                 Position(x=0.7524806261062622, y=-1.3530833721160889, z=0.2888467311859131),
                                 Position(x=0.8615313172340393, y=2.6712381839752197, z=0.24950148165225983),
                                 Position(x=0.7881598472595215, y=2.499725103378296, z=0.27377137541770935),
                                 Position(x=0.6008504629135132, y=2.7838847637176514, z=0.27146992087364197),
                                 Position(x=0.5250619053840637, y=2.599602460861206, z=0.2562762498855591),
                                 Position(x=-0.8268868327140808, y=-0.42351624369621277, z=0.18796001374721527),
                                 Position(x=-0.8045069575309753, y=-0.3426115810871124, z=0.16439802944660187),
                                 Position(x=-0.8931643962860107, y=-0.4159773290157318, z=0.16380201280117035),
                                 Position(x=0.3071064054965973, y=1.693137288093567, z=0.1792670041322708),
                                 Position(x=0.30719614028930664, y=1.7555233240127563, z=0.15431636571884155),
                                 Position(x=0.4096698760986328, y=1.7208905220031738, z=0.15492025017738342),
                                 Position(x=-0.645495593547821, y=-0.9943488240242004, z=0.26637980341911316),
                                 Position(x=-0.5272716283798218, y=-1.2692276239395142, z=0.29252272844314575),
                                 Position(x=-0.5399668216705322, y=-0.9854581952095032, z=0.26634693145751953),
                                 Position(x=-0.7380496263504028, y=-0.996252715587616, z=0.26576435565948486),
                                 Position(x=-0.7233214974403381, y=-1.2827075719833374, z=0.26748332381248474),
                                 Position(x=0.46581923961639404, y=0.5918686389923096, z=0.26094764471054077),
                                 Position(x=0.4655987620353699, y=0.519455075263977, z=0.2603091299533844),
                                 Position(x=0.2780880331993103, y=0.5167971253395081, z=0.2596586048603058),
                                 Position(x=0.2697637677192688, y=0.7931483387947083, z=0.2662479281425476),
                                 Position(x=0.4637969136238098, y=0.7951851487159729, z=0.2565675377845764)])]
)
