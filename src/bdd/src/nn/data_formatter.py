class DataFormatter():

    input_data_buffer = None  #TBD if needed
    output_data_buffer = None #TBD if needed

    def convert_input_image(image):
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

        

    def format_input_data(input_data):
        # stack images correctly
        # correct number of channels and all that stuff
        return input_data_buffer


    def format_output_data(output_data):
        # choose which timeframes of steer and throttle to use
        # or whether to do smoothing or whatnot
        return output_data_buffer


