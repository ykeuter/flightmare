class Mlp:
    def __init__(self):
        self.hidden_layers_weights = []
        self.output_layer_weights = None

    def activate(self):
        pass

    def to_msg(self):
        pass

    @staticmethod
    def from_msg():
        pass

    @staticmethod
    def from_cppn(
        genome,
        config,
        input_coords,
        device="cpu",
    ):
        nodes = create_cppn(
            genome,
            config,
            ["x_in", "y_in", "z_in", "x_out", "Y_out", "z_out"],
            ["weight", "bias"],
        )

        return Mlp(
            nodes,
            input_coords,
            device=device,
        )
