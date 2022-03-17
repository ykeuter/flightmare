import torch

class Mlp:
    def __init__(self, weight_node, bias_node, coords, device):
        self.device = device
        self.weight_node = weight_node
        self.bias_node = bias_node
        self.coords = [
            torch.tensor(c, dtype=torch.float32, device=device)
            for c in coords
        ]
        self.weights = None
        self.biases = None
        self.activation = tanh_activation


    def reset(self):
        self.weights = []
        self.biases = []
        with torch.no_grad():
            bias_coords = torch.zeros((1, 2), dtype=torch.float32, device=self.device)
            for in_coords, out_coords in zip(self.coords[:-1], self.coords[1:]):
                (x_out, y_out, z_out), (x_in, y_in, z_in) = \
                    self._get_coord_inputs(in_coords, out_coords)
                self.weights.append(self.weight_node(
                    x_in=x_in, y_in=y_in, z_in=z_in, x_out=x_out, y_out=y_out, z_out=z_out,
                ))
                (x_out, y_out, z_out), (x_in, y_in, z_in) = \
                    self._get_coord_inputs(bias_coords, out_coords)
                self.biases.append(self.bias_node(
                    x_in=x_in, y_in=y_in, z_in=z_in, x_out=x_out, y_out=y_out, z_out=z_out,
                ))

    def _get_coord_inputs(self, in_coords, out_coords):
        n_in = len(in_coords)
        n_out = len(out_coords)

        x_out = out_coords[:, 0].unsqueeze(1).expand(n_out, n_in)
        y_out = out_coords[:, 1].unsqueeze(1).expand(n_out, n_in)
        z_out = out_coords[:, 2].unsqueeze(1).expand(n_out, n_in)
        x_in = in_coords[:, 0].unsqueeze(0).expand(n_out, n_in)
        y_in = in_coords[:, 1].unsqueeze(0).expand(n_out, n_in)
        z_in = in_coords[:, 2].unsqueeze(0).expand(n_out, n_in)

        return (x_out, y_out, z_out), (x_in, y_in, z_in)

    def activate(self, inputs):
        with torch.no_grad():
            x = torch.tensor(inputs, dtype=torch.float32, device=self.device).unsqueeze(1)
            for w, b in zip(self.weights, self.biases):
                x = self.activation(w.mm(x) + b)
        return x.squeeze(1)

    def to_msg(self):
        pass

    @staticmethod
    def from_msg():
        pass

    @staticmethod
    def from_cppn(
        genome,
        config,
        coords,
        device="cpu",
    ):
        nodes = create_cppn(
            genome,
            config,
            ["x_in", "y_in", "z_in", "x_out", "y_out", "z_out"],
            ["weight", "bias"],
        )

        return Mlp(
            weight_node=nodes[0],
            bias_node=nodes[1],
            coords=coords,
            device=device,
        )
