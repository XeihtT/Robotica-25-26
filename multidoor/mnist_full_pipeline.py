# mnist_full_pipeline.py
import torch
import torch.nn as nn
import torch.optim as optim
import torchvision
import torchvision.transforms as transforms


# ------------------------------------------------------
# 1. Definir CNN de 2 capas convolucionales
# ------------------------------------------------------
class SimpleCNN(nn.Module):
    def __init__(self):
        super(SimpleCNN, self).__init__()

        self.conv1 = nn.Conv2d(1, 16, kernel_size=5, padding=2)
        self.conv2 = nn.Conv2d(16, 32, kernel_size=5, padding=2)
        self.pool = nn.MaxPool2d(2, 2)
        self.fc = nn.Linear(32 * 7 * 7, 10)

    def forward(self, x):
        x = self.pool(torch.relu(self.conv1(x)))   # 16×14×14
        x = self.pool(torch.relu(self.conv2(x)))   # 32×7×7
        x = x.view(-1, 32 * 7 * 7)
        return self.fc(x)


# ------------------------------------------------------
# 2. Entrenamiento
# ------------------------------------------------------
def train_model(batch_size=64, epochs=5, lr=0.001):
    print("🔄 Loading MNIST training dataset...")
    transform = transforms.ToTensor()

    trainset = torchvision.datasets.MNIST(
        root="./data", train=True, download=True, transform=transform
    )
    trainloader = torch.utils.data.DataLoader(
        trainset, batch_size=batch_size, shuffle=True
    )

    model = SimpleCNN()
    criterion = nn.CrossEntropyLoss()
    optimizer = optim.Adam(model.parameters(), lr=lr)

    print("🚀 Training CNN...")
    for epoch in range(epochs):
        running_loss = 0.0
        for images, labels in trainloader:
            optimizer.zero_grad()
            outputs = model(images)
            loss = criterion(outputs, labels)
            loss.backward()
            optimizer.step()
            running_loss += loss.item()

        print(f"Epoch {epoch+1}/{epochs} | Loss = {running_loss/len(trainloader):.4f}")

    # -------------------------------
    # Guardar para Python (opcional)
    # -------------------------------
    print("💾 Saving model state_dict to my_network.pth ...")
    torch.save(model.state_dict(), "my_network.pth")

    # -------------------------------
    # Guardar para C++ (TorchScript)
    # -------------------------------
    print("🎯 Saving TorchScript model to my_network_ts.pt ...")
    model.eval()
    example = torch.rand(1, 1, 28, 28)
    traced = torch.jit.trace(model, example)
    traced.save("my_network_ts.pt")

    print("✅ TorchScript model saved!")

    return model


# ------------------------------------------------------
# 3. Test del modelo
# ------------------------------------------------------
def test_model():
    print("🔍 Loading MNIST test dataset...")
    transform = transforms.ToTensor()

    testset = torchvision.datasets.MNIST(
        root="./data", train=False, download=True, transform=transform
    )
    testloader = torch.utils.data.DataLoader(testset, batch_size=1000, shuffle=False)

    model = SimpleCNN()
    model.load_state_dict(torch.load("my_network.pth", map_location="cpu"))
    model.eval()

    correct = 0
    total = 0

    print("🧪 Evaluating CNN...")
    with torch.no_grad():
        for images, labels in testloader:
            outputs = model(images)
            _, predicted = torch.max(outputs, 1)
            total += labels.size(0)
            correct += (predicted == labels).sum().item()

    accuracy = 100 * correct / total
    print(f"📊 Test accuracy: {accuracy:.2f}%")

    return accuracy


# ------------------------------------------------------
# 4. Ejecutar desde consola
# ------------------------------------------------------
if __name__ == "__main__":
    import sys

    if len(sys.argv) == 1:
        print("Uso:")
        print("  python mnist_full_pipeline.py train   # Entrenar modelo y guardar TorchScript")
        print("  python mnist_full_pipeline.py test    # Probar modelo en Python")
        sys.exit(0)

    command = sys.argv[1]

    if command == "train":
        train_model()
    elif command == "test":
        test_model()
    else:
        print("Comando no válido. Usa 'train' o 'test'.")
