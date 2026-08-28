import QtQuick
import QtQuick3D

View3D {
    id: view
    anchors.fill: parent

    environment: SceneEnvironment {
        clearColor: "#111111"
        backgroundMode: SceneEnvironment.Color
    }

    PerspectiveCamera {
        id: camera
        z: 80
    }

    DirectionalLight {
        eulerRotation: Qt.vector3d(-30, -45, 0)
    }

    Node {
        id: gimbal
        // Применяем вращение от сенсоров
        eulerRotation: Qt.vector3d(sensorData.pitch, -sensorData.yaw, sensorData.roll)

        // ===== Ваша 3D модель =====
        // Model {
        //     id: airbagModel
        //     // Путь к файлу. Если файл лежит в папке assets, то "assets/drone.glb"
        //     source: "airbag.glb" 
        //     
        //     // Масштабирование, если модель слишком большая или маленькая
        //     scale: Qt.vector3d(10, 10, 10)
        //     
        //     // Позиционирование относительно центра вращения (gimbal)
        //     position: Qt.vector3d(0, 0, 0)
        //     
        //     // Материал можно не задавать, если он уже есть внутри .glb файла
        //     // materials: PrincipledMaterial { baseColor: "white" }
        // }

        // ===== Оси координат (оставляем для наглядности) =====
        
        // Ось X (красная)
        Model {
            source: "#Cylinder"
            scale: Qt.vector3d(0.01, 0.2, 0.01)
            position: Qt.vector3d(7.5, 0, 0)
            eulerRotation: Qt.vector3d(0, 0, -90)
            materials: PrincipledMaterial { baseColor: "red" }
        }
        Model {
            source: "#Cone"
            scale: Qt.vector3d(0.02, 0.02, 0.02)
            position: Qt.vector3d(18, 0, 0)
            eulerRotation: Qt.vector3d(0, 0, -90)
            materials: PrincipledMaterial { baseColor: "red" }
        }

        // Ось Y (зелёная)
        Model {
            source: "#Cylinder"
            scale: Qt.vector3d(0.01, 0.2, 0.01)
            position: Qt.vector3d(0, 7.5, 0)
            materials: PrincipledMaterial { baseColor: "green" }
        }
        Model {
            source: "#Cone"
            scale: Qt.vector3d(0.02, 0.02, 0.02)
            position: Qt.vector3d(0, 18, 0)
            materials: PrincipledMaterial { baseColor: "green" }
        }

        // Ось Z (синяя)
        Model {
            source: "#Cylinder"
            scale: Qt.vector3d(0.01, 0.2, 0.01)
            position: Qt.vector3d(0, 0, 7.5)
            eulerRotation: Qt.vector3d(90, 0, 0)
            materials: PrincipledMaterial { baseColor: "blue" }
        }
        Model {
            source: "#Cone"
            scale: Qt.vector3d(0.02, 0.02, 0.02)
            position: Qt.vector3d(0, 0, 18)
            eulerRotation: Qt.vector3d(90, 0, 0)
            materials: PrincipledMaterial { baseColor: "blue" }
        }
    }
}
