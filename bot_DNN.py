from bot import *

class aux_bot_DNN(aux_bot):

    def __init__(self,drive_path = None, pos_path = None, motor_path = None,
                 train_forward = False, train_inverse = False, normalize = True, base_model = None, weighted=True):

        # Size attributes
        MAX_SEQ_LENGTH = 1
        BATCH_SIZE = 16 # originally 16

        super().__init__(drive_path, pos_path, motor_path,
                 train_forward, train_inverse,
                 normalize,BATCH_SIZE,MAX_SEQ_LENGTH,model_type='DNN',weighted=weighted)

        # Constant for forward model
        self.EPOCHS = 100
        self.LEARNING_RATE = 0.0005 #originally 0.0005
        self.MOMENTUM = 0.9
        self.WEIGHT_DECAY = 0 #originally 0
        self.DROPOUT = 0
        self.LAYERS = 1
        self.GAMMA = 0.9
        print("data")
        print(self.motor_slice)

        # Train forward model
        if train_forward:
            self.fk_net = self.forward_net(hidden=1024,inputs=self.input_size,outputs=self.output_size)
            self.train_forward(self.fk_net,self.EPOCHS,self.LEARNING_RATE,self.MOMENTUM,self.WEIGHT_DECAY,annealing=True)
            self.eval_model_forward(self.fk_net,self.fk_train_loss)

        # Load forward model instead
        if not train_forward:
            self.fk_net = self.forward_net(hidden=1024)
            self.fk_net.load_state_dict(torch.load(drive_path + '/models/DNN_forward_2024_02_02-18_25_58_5.155mm',map_location=self.device))
            self.fk_net.eval()
            print("[aux_bot_DNN] Forward model succesfully loaded")

         # Constant for inverse model
        self.EPOCHS = 50 # originally 50
        self.LEARNING_RATE = 0.001 #originally 0.001
        self.MOMENTUM = 0.9
        self.DROPOUT = 0
        self.LAYERS = 1
        self.GAMMA = 0.9

        # Train inverse model
        if train_inverse:
            self.ik_net = self.inverse_net(hidden=1600,inputs=self.output_size,outputs=self.input_size)

            # Retrain base model
            if base_model:
                self.ik_net.load_state_dict(torch.load(drive_path + base_model, map_location=self.device))
            
            self.train_inverse(self.ik_net,self.EPOCHS,self.LEARNING_RATE,self.MOMENTUM,self.WEIGHT_DECAY,annealing=True)
            self.eval_model_inverse(self.ik_net,self.ik_train_loss)

        # Load inverse model instead
        if not train_inverse:
            if weighted:
                self.ik_net = self.inverse_net(hidden=1600,inputs=self.output_size,outputs=self.input_size)
                model_path = drive_path + 'models/DNN_finalized_models/feb4_no_wobble_data_7.931mm'
                self.ik_net.load_state_dict(torch.load(model_path, map_location=self.device))
                self.ik_net.eval()
                self.ik_net.to(self.device)
                print("[aux_bot_DNN] Weighted inverse model succesfully loaded")
                print("[aux_bot_DNN] Model path: ", model_path)
            else:
                self.ik_net = self.inverse_net(hidden=1600,inputs=self.output_size,outputs=self.input_size,num_hidden_layers=1)
                self.ik_net.load_state_dict(torch.load(drive_path + '/models/DNN_inverse_2024_02_21-16_08_24_10.006mm', map_location=self.device))
                self.ik_net.eval()
                self.ik_net.to(self.device)
                print("[aux_bot_DNN] Non-weighted inverse model succesfully loaded")

    ########################
    # FORWARD NETWORK
    ########################

    class forward_net(nn.Module):
        def __init__(self, inputs=9, hidden=1024, outputs=7):
            super().__init__()
            self.fc1 = nn.Linear(inputs,hidden)
            self.fc2 = nn.Linear(hidden,hidden)
            self.fc3 = nn.Linear(hidden,outputs)
            self.input_size = inputs
            self.output_size = outputs

        def forward(self,x):
            x = torch.flatten(x, 1)
            x = self.fc1(x)
            x = F.relu(x)
            x = self.fc2(x)
            x = F.relu(x)
            x = self.fc3(x)
            return x

    #########################
    ## INVERSE NETWORK
    #########################

    class inverse_net(nn.Module):
        def __init__(self, inputs=9, hidden=1024, outputs=7, num_hidden_layers=2):
            print(num_hidden_layers)
            super().__init__()
            self.input_size = inputs
            self.output_size = outputs
            self.hidden_size = hidden
            self.num_hidden_layers = num_hidden_layers
            
            # Define the input layer
            self.fc1 = nn.Linear(inputs, hidden)

            # Dynamically create hidden layers as self.fc2, self.fc3, etc.
            for i in range(2, num_hidden_layers + 2):  # Start from fc2
                setattr(self, f'fc{i}', nn.Linear(hidden, hidden))
            
            # Define the output layer
            setattr(self, f'fc{num_hidden_layers + 2}', nn.Linear(hidden, outputs))

        def forward(self, y):
            y = torch.flatten(y, 1)
            
            # Pass through the input layer
            y = self.fc1(y)
            y = F.relu(y)

            # Pass through the hidden layers dynamically
            for i in range(2, self.num_hidden_layers + 2):  # From fc2 to fc(num_hidden_layers + 1)
                layer = getattr(self, f'fc{i}')
                y = layer(y)
                y = F.relu(y)
            
            # Pass through the output layer
            output_layer = getattr(self, f'fc{self.num_hidden_layers + 2}')
            y = output_layer(y)
            return y

    # class inverse_net(nn.Module):
    #     def __init__(self, inputs=9, hidden=1024, outputs=7):
    #         super().__init__()
    #         self.fc1 = nn.Linear(inputs,hidden)
    #         self.fc2 = nn.Linear(hidden,hidden)
    #         self.fc3 = nn.Linear(hidden,hidden)
    #         self.fc4 = nn.Linear(hidden,outputs)
    #         self.input_size = inputs
    #         self.output_size = outputs

    #     def forward(self,y):
    #         y = torch.flatten(y, 1)
    #         y = self.fc1(y)
    #         y = F.relu(y)
    #         y = self.fc2(y)
    #         y = F.relu(y)
    #         y = self.fc3(y)
    #         y = F.relu(y)
    #         y = self.fc4(y)
    #         return y