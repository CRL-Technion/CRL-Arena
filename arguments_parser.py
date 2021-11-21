class ArgumentsParser:
    def __init__(self, parser):
        # grid args
        parser.add_argument("-c", "--cell", type=float, help="Sets a grid cell size (in meters), default is 0.3.")
        parser.add_argument("-he", "--height", type=float, help="")  # TODO: complete
        parser.add_argument("-w", "--width", type=float, help="")  # TODO: complete

        # scenario args
        parser.add_argument("-g", "--goals", help="Goals location file name for loading a defined scenario."
                                                  " If not specified, goal locations are being "
                                                  "generated randomly and saved under a new file with the "
                                                  "same name. If not specified, default is 'map.map'")
        parser.add_argument("-m", "--map", help="A name for the map (.map) file that is generated for the given run.")
        parser.add_argument("-s", "--scene", help="A name for the scenario (.scene) file that is generated for the "
                                                     "given run. If not specified, default is 'scene.scene'")

        # solver args
        parser.add_argument("-S", "--solver", help="A complete command for executing the MAPF solver, "
                                                  "default behavior is to run a vanilla CBS solver")

        args = parser.parse_args()

        self.cell_size = 0.3 if not args.cell else args.cell
        #self.height = 0.3 if not args.height else args.height
        #self.width = 0.3 if not args.width else args.width

        self.goals = "random" if not args.goals else args.goals
        self.map = "map.map" if not args.map else args.map
        self.scene = "scene.scene" if not args.scene else args.scene
        self.solver = "default" if not args.solver else args.solver
