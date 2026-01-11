#include <SDL3/SDL.h>
#include <GL/glew.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

// ImGui
#include "imgui.h"
#include "imgui_impl_sdl3.h"
#include "imgui_impl_opengl3.h"

// Project Headers
#include "Matrix4x4.hpp"
#include "utils/Mesh.hpp"        
#include "utils/GraphicsUtils.hpp" 

// CLASE TRANSFORM:
// Representa la posición, rotación y escala de un objeto en el espacio local.
class Transform {
public:
    Vec3 position = { 0, 0, 0 };
    Vec3 rotationEuler = { 0, 0, 0 }; // // Rotación en grados (X, Y, Z) para editar en la UI
    Vec3 scale = { 1, 1, 1 };

    // Convierte los datos (TRS) en una Matriz 4x4 Local.
    // Orden: Traslación * Rotación * Escala
    Matrix4x4 GetLocalMatrix() const {
        return Matrix4x4::FromTRS(position, Quat::FromEulerZYX(rotationEuler. z,rotationEuler.y,rotationEuler.x), scale);
    }
};
// CLASE GAMEOBJECT:
// Es un nodo en el "Grafo de Escena" (Scene Graph).
// Permite crear jerarquias (padres e hijos).
class GameObject {
public:
    std::string name = "New Object";
    Transform transform; // Su posición local respecto al padre
    GameObject* parent = nullptr; // Puntero al padre (si es null, es raíz)
    std::vector<GameObject*> children;// Lista de hijos

    // Calcula la Matriz Global (World Matrix) recursivamente.
     // Si tiene padre, multiplica la matriz global del padre por la local de este objeto.
     // Esto hace que si mueves al padre, los hijos se muevan con él.
    Matrix4x4 GetGlobalMatrix() const {
        if (parent == nullptr) {
            return transform.GetLocalMatrix();
        }
        // Matriz Global = Matriz Global del Padre * Matriz Local del Hijo
        return parent->GetGlobalMatrix().Multiply(transform.GetLocalMatrix());
    }
    // Añade un hijo a este objeto y establece la relacion bidireccional
    void AddChild(GameObject* child) {
        if (child) {
            child->parent = this;
            children.push_back(child);
        }
    }
};
// CLASE CAMERA:
// Gestiona la proyección y la vista de la escena.
class Camera {
public:
    Vec3 position = { 0, 0, 5 };
    Vec3 rotation = { 0, 0, 0 };
    float fov = 45.0f; // Campo de visión
    float nearPlane = 0.1f;// Distancia mínima de renderizado
    float farPlane = 100.0f;// Distancia máxima de renderizado
    float aspectRatio = 1.77f;// Relación de aspecto (Ancho / Alto) 

    // Matriz de Vista (View Matrix):
     // Es la INVERSA de la transformación de la cámara.
     // Mover la cámara a la derecha equivale a mover todo el mundo a la izquierda.
    Matrix4x4 GetViewMatrix() const {
        Matrix4x4 camGlobal = Matrix4x4::Translate(position).Multiply(Matrix4x4::Rotate(Quat::FromEulerZYX(rotation.z,rotation.y,rotation.x)));
        return camGlobal.InverseTR();// Invierte Traslación y Rotación
    }

    // Matriz de Proyección:
    // Convierte el espacio 3D en coordenadas 2D de pantalla con perspectiva.
    Matrix4x4 GetProjectionMatrix() const {
        Matrix4x4 res = {};
        float tanHalfFov = tan(fov * 0.5f * (3.14159f / 180.0f));
        // Fórmulas estándar de proyección OpenGL
        res.At(0, 0) = 1.0f / (aspectRatio * tanHalfFov);
        res.At(1, 1) = 1.0f / tanHalfFov;
        res.At(2, 2) = -(farPlane + nearPlane) / (farPlane - nearPlane);
        res.At(2, 3) = -1.0f;
        res.At(3, 2) = -(2.0f * farPlane * nearPlane) / (farPlane - nearPlane);
        return res;
    }
};
// -----------------------------------------------------------------------------
// 3. HELPERS DE SHADERS
// -----------------------------------------------------------------------------
// Lee el código fuente del shader desde un archivo de texto
std::string LoadShaderFile(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open shader file: " << filepath << std::endl;
        return "";
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

// Compila un shader individual (Vertex o Fragment) y verifica errores
GLuint CompileShader(GLenum type, const std::string& source) {
    const char* srcPtr = source.c_str();
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &srcPtr, nullptr);
    glCompileShader(shader);

    int success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetShaderInfoLog(shader, 512, nullptr, infoLog);
        std::cerr << "ERROR::SHADER::COMPILATION_FAILED\n" << infoLog << std::endl;
    }
    return shader;
}

// Crea el programa final linkando Vertex y Fragment shaders
GLuint CreateShaderProgram(const std::string& vertPath, const std::string& fragPath) {
    std::string vertCode = LoadShaderFile(vertPath);
    std::string fragCode = LoadShaderFile(fragPath);

    if (vertCode.empty() || fragCode.empty()) return 0;

    GLuint vertexShader = CompileShader(GL_VERTEX_SHADER, vertCode);
    GLuint fragmentShader = CompileShader(GL_FRAGMENT_SHADER, fragCode);

    GLuint shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);

    int success;
    glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetProgramInfoLog(shaderProgram, 512, nullptr, infoLog);
        std::cerr << "ERROR::PROGRAM::LINKING_FAILED\n" << infoLog << std::endl;
    }

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
    return shaderProgram;
}

// -----------------------------------------------------------------------------
// UI: (TODO)
// -----------------------------------------------------------------------------

// Variable global para saber qué objeto estamos editando en el Inspector
GameObject* selectedObject = nullptr;

// DIBUJADO DE LA JERARQUÍA (UI):
// Función recursiva que dibuja el árbol de objetos en la ventana "Hierarchy" de ImGui.
void DrawHierarchyNode(GameObject* node) {
if (!node) return;

// Configura flags para el nodo del árbol (si está seleccionado, si es hoja, etc.)
ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_OpenOnDoubleClick;
if (node == selectedObject) flags |= ImGuiTreeNodeFlags_Selected;
if (node->children.empty()) {
    flags |= ImGuiTreeNodeFlags_Leaf;
}

// Dibuja el nodo
bool nodeOpen = ImGui::TreeNodeEx((void*)(intptr_t)node, flags, "%s", node->name.c_str());

// Si hacemos click, lo marcamos como seleccionado
if (ImGui::IsItemClicked()) selectedObject = node;
// Si el nodo está abierto, dibujamos recursivamente a sus hijos
if (nodeOpen) {
 
    for (auto* child : node->children) {
        DrawHierarchyNode(child);
    }
    ImGui::TreePop(); 
}
}

// -----------------------------------------------------------------------------
// RENDER (TODO)
// -----------------------------------------------------------------------------
// Función recursiva que dibuja los cubos reales en OpenGL.
void RenderNode(GameObject* node, GLuint shaderProgram, const Matrix4x4& view, const Matrix4x4& proj, Mesh& mesh) {
    if (!node) return;

// 1. Calcular la matriz Model (Global) del objeto actual.
//    Esto combina las transformaciones de todos sus padres.
    Matrix4x4 model = node->GetGlobalMatrix();

// 2. Enviar las matrices MVP (Model, View, Projection) al Shader.
//    El shader multiplicará Vertice * Model * View * Projection.
    GraphicsUtils::UploadMVP(shaderProgram, model, view, proj);

// 3. Enviar color blanco por defecto
    GraphicsUtils::UploadColor(shaderProgram, Vec3(1.0, 1.0, 1.0));

// 4. Dibuja la geometría (el cubo)
    mesh.Draw();

// 5. Renderizar recursivamente a los hijos
    for (auto* child : node->children) {
        RenderNode(child, shaderProgram, view, proj, mesh);
    }
}

// -----------------------------------------------------------------------------
// MAIN (TODO)
// -----------------------------------------------------------------------------
int main(int argc, char** argv) {
    // 1. Setup SDL & OpenGL
    // Crea la ventana y el contexto gráfico.
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        std::cerr << "SDL_Init Error: " << SDL_GetError() << std::endl;
        return 1;
    }

    
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);

    SDL_Window* window = SDL_CreateWindow("Project: Mini-Scene 3D", 1280, 720, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
    if (!window) return 1;

    SDL_GLContext glContext = SDL_GL_CreateContext(window);
    SDL_GL_MakeCurrent(window, glContext);
    SDL_GL_SetSwapInterval(1); // VSync

    if (glewInit() != GLEW_OK) return 1;

    glEnable(GL_DEPTH_TEST);

    // 2. INICIALIZACIÓN DE IMGUI
    // Configura el sistema de UI.
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();
    ImGui_ImplSDL3_InitForOpenGL(window, glContext);
    ImGui_ImplOpenGL3_Init("#version 330");

    // 3. RECURSOS
    // Crea el cubo y compila los shaders.
    Mesh cubeMesh;
    cubeMesh.InitCube();
    //TODO: Assegureu-vos de tenir els fitxers vs.glsl i fs.glsl al mateix nivell de l'executable
    GLuint shaderProgram = CreateShaderProgram("vs.glsl", "fs.glsl");
    if (shaderProgram == 0) std::cerr << "Warning: Shaders not loaded properly." << std::endl;

    // 4. ESCENA INICIAL
    // Crea un objeto raíz y configura la cámara por defecto.
    GameObject* rootObject = new GameObject();
    std::vector<GameObject*> sceneRoots = { rootObject };

	Camera mainCamera; //TODO: Inicialitzar la camara
    mainCamera.position = { 0, 0, 10 };
    mainCamera.fov = 45.0f;
    mainCamera.nearPlane = 0.1f;
    mainCamera.farPlane = 100.0f;


	// 5. Loop Principal
    bool running = true;
    while (running) {
        // --- INPUT ---
        // Procesa eventos de cerrar ventana y pasa eventos a ImGui
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL3_ProcessEvent(&event);
            if (event.type == SDL_EVENT_QUIT) running = false;
            if (event.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED && event.window.windowID == SDL_GetWindowID(window)) running = false;
        }

        // --- UPDATE UI ---
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL3_NewFrame();
        ImGui::NewFrame();

        // UI: Jerarquia
        ImGui::Begin("Hierarchy");
        if (ImGui::Button("Add Object to Root"))
        {
            // Lógica para crear nuevo objeto
            GameObject* newObj = new GameObject();
            newObj->name = "Object " + std::to_string(sceneRoots.size());
            sceneRoots.push_back(newObj); 
        }
        ImGui::Separator();
        for (auto* obj : sceneRoots) DrawHierarchyNode(obj);
        ImGui::End();

        // UI: Inspector
        // Permite editar Transform del objeto seleccionado
        ImGui::Begin("Inspector");
        if (selectedObject) {
            // Sliders para modificar Posición, Rotación y Escala en tiempo real
            ImGui::Text("Selected: %s", selectedObject->name.c_str());
            ImGui::Separator();

            // Posición
            float pos[3] = { (float)selectedObject->transform.position.x, (float)selectedObject->transform.position.y, (float)selectedObject->transform.position.z };
            if (ImGui::DragFloat3("Position", pos, 0.1f)) {
                selectedObject->transform.position = { (double)pos[0], (double)pos[1], (double)pos[2] };
            }

            // Rotación
            float rot[3] = { (float)selectedObject->transform.rotationEuler.x, (float)selectedObject->transform.rotationEuler.y, (float)selectedObject->transform.rotationEuler.z };
            if (ImGui::DragFloat3("Rotation (Euler)", rot, 0.5f)) {
                selectedObject->transform.rotationEuler = { (double)rot[0], (double)rot[1], (double)rot[2] };
            }

            // Escala
            float scl[3] = { (float)selectedObject->transform.scale.x, (float)selectedObject->transform.scale.y, (float)selectedObject->transform.scale.z };
            if (ImGui::DragFloat3("Scale", scl, 0.1f)) {
                selectedObject->transform.scale = { (double)scl[0], (double)scl[1], (double)scl[2] };
            }

            ImGui::Separator();

            // Botón para añadir hijo al objeto seleccionado
            if (ImGui::Button("Add Child")) {
                GameObject* newChild = new GameObject();
                newChild->name = "Child of " + selectedObject->name;
                selectedObject->AddChild(newChild);
            }
        }
        else {
            ImGui::Text("Select an object from Hierarchy.");
        }
        ImGui::End();

        // UI: Camera
        // Ventana Cámara: Ajustes de FOV y posición de cámara
        ImGui::Begin("Camera Settings");
        float fov = mainCamera.fov;
        if (ImGui::SliderFloat("FOV (Y)", &fov, 10.0f, 170.0f)) {
            mainCamera.fov = fov; 
        }

        float nearP = mainCamera.nearPlane;
        float farP = mainCamera.farPlane;
        
        if (ImGui::DragFloat("Near Plane", &nearP, 0.1f)) mainCamera.nearPlane = nearP;
        if (ImGui::DragFloat("Far Plane", &farP, 1.0f))   mainCamera.farPlane = farP;

        ImGui::Separator();
        ImGui::Text("Camera Transform");

      
        float cPos[3] = { (float)mainCamera.position.x, (float)mainCamera.position.y, (float)mainCamera.position.z };

        if (ImGui::DragFloat3("Pos", cPos, 0.1f))
        {
            mainCamera.position.x = (double)cPos[0];
            mainCamera.position.y = (double)cPos[1];
            mainCamera.position.z = (double)cPos[2];
        }
        ImGui::End();
        // --- RENDER ---
        // Actualiza el tamaño del viewport si la ventana cambia de tamaño
        int w, h;
        SDL_GetWindowSize(window, &w, &h);
        glViewport(0, 0, w, h);
        if (h > 0)
        {
            mainCamera.aspectRatio = (float)w / (float)h;
		
        }
        // Limpia la pantalla (Color y Profundidad)
        glClearColor(0.1f, 0.1f, 0.15f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (shaderProgram != 0) {
            glUseProgram(shaderProgram);
        // Obtiene matrices de la cámara
            Matrix4x4 view = mainCamera.GetViewMatrix();
            Matrix4x4 proj = mainCamera.GetProjectionMatrix();

        // Llama al renderizado recursivo para dibujar toda la escena
            for (auto* obj : sceneRoots) {
                RenderNode(obj, shaderProgram, view, proj, cubeMesh);
            }
        }

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        SDL_GL_SwapWindow(window);
    }

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImGui::DestroyContext();
    glDeleteProgram(shaderProgram);
    SDL_GL_DestroyContext(glContext);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}