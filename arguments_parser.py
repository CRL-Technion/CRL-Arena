class ArgumentsParser:
    def __init__(self, parser):
        # grid args
        parser.add_argument("-c", "--cell", type=float, help="Sets a grid cell size (in meters), default is 0.3.")
        parser.add_argument("-he", "--height", type=float, help="Grid height (in meters), default is 3m")
        parser.add_argument("-w", "--width", type=float, help="Grid width (in meters), default is 6m")

        # scenario args
        parser.add_argument("-g", "--goals", help="Goals location file name for loading defined goals.")
        parser.add_argument("-m", "--map", help="A name for the map (.map) file that is generated for the given run."
                                                "If not specified, default is 'map.map'")
        parser.add_argument("-s", "--scene", help="A name for the scenario (.scen) file that is generated for the "
                                                  "given run. If not specified, default is 'scene.scen'")

        # solver args
        parser.add_argument("-S", "--solver", help="A complete command for executing the MAPF solver, "
                                                   "default behavior is to run a vanilla CBS solver")

        args = parser.parse_args()

        self.cell_size = 0.3 if not args.cell else float(args.cell)
        self.height = 3.0 if not args.height else float(args.height)
        self.width = 6.0 if not args.width else float(args.width)

        self.goals = "" if not args.goals else args.goals
        self.map = "map.map" if not args.map else args.map
        self.scene = "scene.scen" if not args.scene else args.scene
        self.solver = "default" if not args.solver else args.solver
