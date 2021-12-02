from demo_config import DEMO_ARENA_CONFIG


class ArgumentsParser:
    def __init__(self, parser):
        # grid args
        parser.add_argument("-c", "--cell", type=float, help="Sets a grid cell size (in meters), default is 0.3.")
        parser.add_argument("-he", "--height", type=float, help="Grid height (in meters), default is 3m")
        parser.add_argument("-w", "--width", type=float, help="Grid width (in meters), default is 6m")
        parser.add_argument("--config", type=str, help="Choose a built-in arena dimensions from the "
                                                       "option in 'demp_config.py' file. \n"
                                                       "SMALL := 2.5x2.5, MEDIUM := 3x6, LARGE := 4.5x7 (height x width),"
                                                       "with cell size 0.3m.\n"
                                                       "Ignores any other arena configurations that where provided.")

        # scenario args
        parser.add_argument("-g", "--goals", help="Goals location file name for loading defined goals.")
        parser.add_argument("-m", "--map", help="A name for the map (.map) file that is generated for the "
                                                "given execution. If not specified, default is 'map.map'.\n"
                                                "Note that if a file with the same name already exists, "
                                                "it'll be overwritten.")
        parser.add_argument("-s", "--scene", help="A name for the scenario (.scen) file that is generated for the "
                                                  "given execution. If not specified, default is 'scene.scen'.\n"
                                                  "Note that if a file with the same name already exists, "
                                                  "it'll be overwritten.")

        # solver args
        parser.add_argument("-S", "--solver", help="A complete command for executing the MAPF solver, "
                                                   "default behavior is to run a vanilla CBS solver")

        args = parser.parse_args()

        if args.config:
            # choosing from built-in demo arenas
            config = DEMO_ARENA_CONFIG[args.config]
            self.cell_size = config['cell_size']
            self.height = config['height']
            self.width = config['width']
        else:
            self.cell_size = 0.3 if not args.cell else float(args.cell)
            self.height = 3.0 if not args.height else float(args.height)
            self.width = 6.0 if not args.width else float(args.width)

        self.goals = "" if not args.goals else args.goals
        self.map = "map.map" if not args.map else args.map
        self.scene = "scene.scen" if not args.scene else args.scene
        self.solver = "default" if not args.solver else args.solver
