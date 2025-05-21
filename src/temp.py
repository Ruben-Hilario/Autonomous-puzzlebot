# Importar las bibliotecas necesarias
import numpy as np
from sklearn.datasets import make_classification
from sklearn.model_selection import train_test_split
from sklearn.neighbors import KNeighborsClassifier
from sklearn.metrics import accuracy_score

# 1. Generar datos sintéticos (para ejemplo)
X, y = make_classification(
    n_samples=100,  # 100 muestras
    n_features=4,   # 4 características por muestra
    n_classes=2,    # 2 clases (binario)
    random_state=42
)

# 2. Dividir los datos en entrenamiento (70%) y prueba (30%)
X_train, X_test, y_train, y_test = train_test_split(
    X, y, test_size=0.3, random_state=42
)

# 3. Crear y entrenar el modelo KNN (con k=3)
knn = KNeighborsClassifier(n_neighbors=3)
knn.fit(X_train, y_train)

# 4. Predecir las clases para los datos de prueba
y_pred = knn.predict(X_test)

# 5. Calcular la precisión del modelo
accuracy = accuracy_score(y_test, y_pred)
print(f"Precisión del modelo KNN: {accuracy:.2f}")

# Opcional: Predecir una nueva muestra
new_sample = np.array([[0.5, -1.2, 0.8, 1.5]])
prediction = knn.predict(new_sample)
print(f"Predicción para la nueva muestra: {prediction[0]}")

