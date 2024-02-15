from bot import *

class aux_bot_LSTM(aux_bot):

    def __init__(self,drive_path = None, pos_path = None, motor_path = None,
                 train_forward = False, train_inverse = False, normalize=True):

        # Size attributes
        MAX_SEQ_LENGTH = 100
        BATCH_SIZE = 16

        super().__init__(drive_path, pos_path, motor_path,
                 train_forward, train_inverse,
                 normalize,BATCH_SIZE,MAX_SEQ_LENGTH,model_type='LSTM')

        # Constant for forward model
        self.EPOCHS = 100
        self.LEARNING_RATE = 0.0005
        self.MOMENTUM = 0.9
        self.WEIGHT_DECAY = 0
        self.DROPOUT = 0
        self.LAYERS = 1
        self.GAMMA = 0.95

        # Train forward model
        if  train_forward:
            self.fk_net = self.forward_net(inputs = self.input_size,
                                           hidden_lstm = 512,
                                           hidden = 1552,
                                           outputs = self.output_size,
                                           num_layers = 1,
                                           dropout = self.DROPOUT,
                                           device = self.device)

            self.fk_net.load_state_dict(torch.load(drive_path + '/models/forward_2024_02_02-16_40_20_3.213mm',map_location=self.device))

            # Train and evaluate
            self.train_forward(self.fk_net,self.EPOCHS,self.LEARNING_RATE,self.MOMENTUM,self.WEIGHT_DECAY,annealing=True)
            self.eval_model_forward(self.fk_net,self.fk_train_loss)

        # Load forward model instead
        if not train_forward:
            self.fk_net = self.forward_net(hidden_lstm=512, hidden=1552)
            self.fk_net.load_state_dict(torch.load(drive_path + '/models/forward_2024_02_02-16_40_20_3.213mm',map_location=self.device))
            self.fk_net.eval()
            print("[aux_bot_LSTM] Forward model succesfully loaded")

        # Constant for inverse model
        self.EPOCHS = 100
        self.LEARNING_RATE = 0.001
        self.MOMENTUM = 0.9
        self.DROPOUT = 0
        self.LAYERS = 1
        self.GAMMA = 0.95

        # Train inverse model
        if  train_inverse:
            self.ik_net = self.inverse_net(hidden=1552,
                                           hidden_lstm = 512,
                                           inputs=self.output_size,
                                           outputs=self.input_size,
                                           device=self.device,
                                           dropout=self.DROPOUT,
                                           num_layers=self.LAYERS,
                                           linear_depth=3)
            # Bootstrap off previous run
            self.ik_net.load_state_dict(torch.load(drive_path + '/models/LSTM_inverse_2024_02_06-17_45_21_10.445mm',map_location=self.device))

            # Train and evaluate
            self.train_inverse(self.ik_net,self.EPOCHS,self.LEARNING_RATE,self.MOMENTUM,self.WEIGHT_DECAY,annealing=True)
            self.eval_model_inverse(self.ik_net,self.ik_train_loss)

        # Load inverse model instead
        if not train_inverse:
            self.ik_net = self.inverse_net(hidden_lstm=512, hidden=1552,device=self.device,linear_depth=3)
            self.ik_net.load_state_dict(torch.load(drive_path + '/models/LSTM_inverse_2024_02_15-14_37_57_8.007mm',map_location=self.device))
            self.ik_net.eval()
            self.ik_net.to(self.device)
            print("[aux_bot_LSTM] Inverse model succesfully loaded")

    ########################
    # FORWARD NETWORK
    ########################

    class forward_net(nn.Module):
        def __init__(self, inputs=9, hidden_lstm=512, hidden=1024, outputs=7,num_layers=1,dropout=0,device=None):
            super().__init__()
            self.device = device

            self.lstm = nn.LSTM(
            input_size=inputs,
            hidden_size=hidden_lstm,
            batch_first=True,
            num_layers=num_layers,
            )

            self.fc1 = nn.Linear(2 * hidden_lstm, hidden)
            self.fc2 = nn.Linear(hidden, hidden)
            self.fc3 = nn.Linear(hidden, outputs)

            self.input_size = inputs
            self.hidden_size = hidden
            self.hidden_lstm_size = hidden_lstm
            self.output_size = outputs
            self.num_layers = num_layers
            self.dropout = nn.Dropout(dropout)

        def forward(self,x):
            batch_size = x.shape[0]
            h0 = torch.zeros(self.num_layers, batch_size, self.hidden_lstm_size).requires_grad_().to(self.device)
            c0 = torch.zeros(self.num_layers, batch_size, self.hidden_lstm_size).requires_grad_().to(self.device)

            _, (hn, cn) = self.lstm(x, (h0, c0))

            y = torch.cat((hn[-1], cn[-1]), dim=1)

            y = F.relu(y)
            y = self.fc1(y)
            y = F.relu(y)
            y = self.fc2(y)
            y = F.relu(y)
            y = self.fc3(y)

            return y

    #########################
    ## INVERSE NETWORK
    #########################

    class inverse_net(nn.Module):
        def __init__(self, inputs=7, hidden_lstm=512, hidden=1024, outputs=9, num_layers=1, dropout=0.1, device=None, linear_depth = None):
            super().__init__()

            self.device = device

            self.lstm = nn.LSTM(
            input_size=inputs,
            hidden_size=hidden_lstm,
            batch_first=True,
            num_layers=num_layers,
            )

            self.drop = nn.Dropout(dropout)

            # Default size
            if linear_depth == None:
                self.fc1 = nn.Linear(hidden_lstm,hidden)
                self.fc2 = nn.Linear(hidden,hidden)
                self.fc3 = nn.Linear(hidden,outputs)

            # Explore different sizes
            else:
                self.linear_layers = nn.ModuleList()  # Use nn.ModuleList
                if linear_depth == 1:
                    self.linear_layers.append(nn.Linear(2 * hidden_lstm, outputs))
                else:
                    self.linear_layers.append(nn.Linear(2 * hidden_lstm, hidden))
                    for _ in range(1, linear_depth - 1):
                        self.linear_layers.append(nn.Linear(hidden, hidden))
                    self.linear_layers.append(nn.Linear(hidden, outputs))

            self.input_size = inputs
            self.hidden_size = hidden
            self.hidden_lstm_size = hidden_lstm
            self.output_size = outputs
            self.num_layers = num_layers
            self.dropout = dropout
            self.linear_depth = linear_depth

        def forward(self,y):

            batch_size = y.shape[0]
            h0 = torch.zeros(self.num_layers, batch_size, self.hidden_lstm_size).requires_grad_().to(self.device)
            c0 = torch.zeros(self.num_layers, batch_size, self.hidden_lstm_size).requires_grad_().to(self.device)

            _, (hn, cn) = self.lstm(y, (h0, c0))
            x = torch.cat((hn[-1], cn[-1]), dim=1)

            x = self.drop(x)

            # Default architecture
            if self.linear_depth == None:
                x = F.relu(x)
                x = self.fc1(x)
                x = F.relu(x)
                x = self.fc2(x)
                x = F.relu(x)
                x = self.fc3(x)

            # n-dimensional
            else:
                for fc in self.linear_layers:
                    x = F.relu(x)
                    x = fc(x)

            return x