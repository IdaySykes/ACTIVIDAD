"""
ALUMNA: Beltrán Zepeda Yadira Monserrat
GRUPO: 5-3
MATRIZ DE ROTACIÓNES

NOTA: Me gustaría agregar que en numpy (np.sin, np.cos, np.tan)
las funciones se esperan siempre ángulos en radianes, no en grados

"""
import numpy as np

def rot_x(x: float, y: float, z: float, theta: float) -> np.ndarray:
    
    """
    Rota un punto tridimensional alrededor del eje X.

    Esta función aplica una transformación de rotación al vector (x, y, z) tomando
    como eje de giro el eje X. El ángulo de rotación debe proporcinarse en radianes
    para garantizar la correcta aplicación de la matriz de rotación.

    Parámetros:
    ----------
    x : float
        Coordenada del punto en el eje X.
    y : float
        Coordenada del punto en el eje Y.
    z : float
        Coordenada del punto en el eje Z.
    theta : float
        Ángulo de rotación en radianes.

    Retorna:
    -------
    np.ndarray
        Un arreglo de numpy con las nuevas coordenadas [x', y', z']
        después de aplica la rotación.
    """
    p = np.array([x, y, z])
    R = np.array([
        [1, 0, 0],
        [0, np.cos(theta), -np.sin(theta)],
        [0, np.sin(theta),  np.cos(theta)]
    ])
    return R @ p


def rot_y(x: float, y: float, z: float, theta: float) -> np.ndarray:
    
    """
    Rota un punto tridimensional alrededor del eje Y.

    La función utiliza la matriz de rotación correspondiente al eje Y
    para transformar el vector (x, y, z). El ángulo de rotación debe
    especificarse en radianes.

    Parámetros:
    ----------
    x : float
        Coordenada del punto en el eje X.
    y : float
        Coordenada del punto en el eje Y.
    z : float
        Coordenada del punto en el eje Z.
    theta : float
        Ángulo de rotación en radianes.

    Retorna:
    -------
    np.ndarray
        Vector rotado represetado como [x', y', z'].
    """
    p = np.array([x, y, z])
    R = np.array([
        [ np.cos(theta), 0, np.sin(theta)],
        [ 0, 1, 0],
        [-np.sin(theta), 0, np.cos(theta)]
    ])
    return R @ p


def rot_z(x: float, y: float, z: float, theta: float) -> np.ndarray:
    
    """
    Rota un punto tridimensional alrededor del eje Z.

    Se aplica la matriz de rotación estándar sobre el eje Z al vector
    (x, y, z). El ángulo de rotación se espera en radianes.

    Parámetros:
    ----------
    x : float
        Coordenada del punto en el eje X.
    y : float
        Coordenada del punto en el eje Y.
    z : float
        Coordenada del punto en el eje Z.
    theta : float
        Ángulo de rotación en radianes.

    Retorna:
    -------
    np.ndarray
        Vector rotado con la forma [x', y', z'].
    """
    p = np.array([x, y, z])
    R = np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta),  np.cos(theta), 0],
        [0, 0, 1]
    ])
    return R @ p


def rotar(x: float, y: float, z: float, theta: float, axis: str) -> np.ndarray:
    
    """
    Rota un punto tridimensional alrededor de un eje indicado.

    Esta funcion actúa como envoltura (*wrapper*) de las funciones
    rot_x, rot_y y rot_z. Dependiendo del eje que indique el usuario
    se aplicara la rotación correspondiente. El ángulo debe
    proporcionarse en radianes.

    Parámetros:
    ----------
    x : float
        Coordenada del punto en el eje X.
    y : float
        Coordenada del punto en el eje Y.
    z : float
        Coordenada del punto en el eje Z.
    theta : float
        Ángulo de rotación en radianes.
    axis : str
        Eje de rotación; puede ser 'x', 'y' o 'z' (no distingue mayúsculas/minúsculas).

    Retorna:
    -------
    np.ndarray
        Vector rotado con coordenadas [x', y', z'].

    Excepciones:
    -----------
    ValueError
        Si el eje proporciondo no corresponde a 'x', 'y' o 'z'.
    """
    axis = axis.lower()
    if axis == "x":
        return rot_x(x, y, z, theta)
    elif axis == "y":
        return rot_y(x, y, z, theta)
    elif axis == "z":
        return rot_z(x, y, z, theta)
    else:
        raise ValueError("El eje debe ser 'x', 'y' o 'z'.")


if __name__ == "__main__":
    print("=== Rotación de vectores ===\n")
    datos = input("Introduce el vector en formato 'x y z': ")
    x, y, z = map(float, datos.split())
    eje = input("Introduce el eje de rotación (X, Y o Z): ")
    grados = float(input("Introduce el ángulo en grados: "))

    # Conversión a radianes
    rad = np.radians(grados)

    resultado = rotar(x, y, z, rad, eje)
    print(f"\nVector rotado {grados}° alrededor del eje {eje.upper()}:\n{resultado}")
