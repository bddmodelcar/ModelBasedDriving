class DataProcessor():

    def convert_image(image):
        if self.type == 'pytorch':
            return numpy_int_mat_to_pytorch_float_tensor(image)
        # you can see how this can be extended to other frameworks...
        else:
            return image

    def numpy_int_mat_to_pytorch_float_tensor(np_mat):
        """helper function, incomplete
        """
        tensor = torch.from_numpy(np_mat)
        tensor = (tensor.float() / 255.0) - 0.5
        # TODO: add more processing code here
        return tensor
