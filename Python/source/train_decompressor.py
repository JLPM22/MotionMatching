import torch
from torch import nn
import dataset_decompressor
from torch.utils.data import DataLoader
import time

# Hyperparameters
learning_rate = 1e-3
batch_size = 64
epochs = 100
hidden_size = 512
filename = "data/decompressor_jltest.onnx"

# Device
device = "cuda" if torch.cuda.is_available() else "cpu"
print(f"Using {device} device")

# Datasets
(
    training_data,
    test_data,
    input_size,
    output_size,
) = dataset_decompressor.get_training_and_test_dataset(
    "C:/Users/JLPM/Desktop/Trabajo/TFM/MotionMatching/MotionMatchingUnity/Assets/Animations/MMData/JLData",
    "JLData",
    device,
)

train_dataloader = DataLoader(training_data, batch_size=batch_size, shuffle=True)
test_dataloader = DataLoader(test_data, batch_size=batch_size, shuffle=True)

# Neural Network
class Decompressor(nn.Module):
    def __init__(self, input_size, hidden_size, output_size):
        super(Decompressor, self).__init__()

        self.linear_relu_stack = nn.Sequential(
            nn.Linear(input_size, hidden_size),
            nn.ReLU(),
            nn.Linear(hidden_size, hidden_size),
            nn.ReLU(),
            nn.Linear(hidden_size, hidden_size),
            nn.ReLU(),
            nn.Linear(hidden_size, output_size),
        )

    def forward(self, x):
        return self.linear_relu_stack(x)


model = Decompressor(
    input_size=input_size, hidden_size=hidden_size, output_size=output_size
).to(device)

# Training
def train_loop(dataloader, model, loss_fn, optimizer):
    size = len(dataloader.dataset)
    for batch, (X, y) in enumerate(dataloader):
        # Compute prediction and loss
        y_pred = model(X)
        loss = loss_fn(y_pred, y)

        # Backpropagation
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        # Print progress
        # if batch % 100 == 0:
        #     loss, current = loss.item(), batch * len(X)
        #     print(f"loss: {loss:>7f}  [{current:>5d}/{size:>5d}]")


def test_loop(dataloader, model, loss_fn, debug):
    size = len(dataloader.dataset)
    num_batches = len(dataloader)
    test_loss = 0

    with torch.no_grad():
        for X, y in dataloader:
            pred = model(X)
            test_loss += loss_fn(pred, y).item()

    test_loss /= num_batches
    if debug:
        print(f"Test Error: \n Avg loss: {test_loss:>8f} \n")


loss_fn = nn.MSELoss()
optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)
# optimizer = torch.optim.SGD(model.parameters(), lr=learning_rate)

start_time = time.time()

for t in range(epochs):
    debug = (t + 1) % 20 == 0
    if debug:
        print(f"Epoch {t+1}\n-------------------------------")
    train_loop(train_dataloader, model, loss_fn, optimizer)
    test_loop(test_dataloader, model, loss_fn, debug)

end_time = time.time()

print(f"Done! \nTime: {end_time - start_time:>8f} seconds")

# Save model
torch.onnx.export(
    model,
    torch.randn(1, input_size, device=device),  # dummy input
    filename,
    export_params=True,
    opset_version=9,
    do_constant_folding=True,
    input_names=["X"],
    output_names=["Y"],
)
