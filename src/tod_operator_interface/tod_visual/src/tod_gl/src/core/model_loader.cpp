/**
 * @file model_loader.cpp
 * @brief TODO: Add brief
 * @copyright 2024 TUMFTM
 */

#include "tod_gl/core/model_loader.hpp"

#include <iostream>
#include <fstream>

#include "tod_gl/renderer/renderer.hpp"
#include "tod_gl/scene/components.hpp"
#include "tod_gl/systems/shader_system.hpp"

#include <assimp/postprocess.h>
#include <assimp/Importer.hpp>
#define BOOST_FILESYSTEM_VERSION 3
#define BOOST_FILESYSTEM_NO_DEPRECATED
#include <glm/gtc/matrix_transform.hpp>
#include "stb_image/stb_image.h"

namespace fs = boost::filesystem;

namespace tod_gl {

ModelLoader::ModelLoader(std::shared_ptr<Scene> activeScene, const std::string &modelPath)
    : _active_scene(activeScene), _modelPath(modelPath) {}

Entity ModelLoader::load_model(const std::string &modelName, Entity parent, const glm::vec3 &scaling,
                              const glm::vec3 &rotation, const std::string &packagePath) {
    _modelName = modelName;
    _meshes.clear();

    unsigned int modelShader =
        ShaderSystem::create_shader_program((packagePath + "/resources/shaders/model.vert").c_str(),
                                          (packagePath + "/resources/shaders/model.frag").c_str());

    GLubyte data[] = {255, 255, 255, 255};
    _emptyTexture = Texture(1, 1, "texture_diffuse", GL_TEXTURE_2D, GL_RGB, GL_RGB);
    Renderer::generate_texture(_emptyTexture, data, modelShader, 0);

    // Get File directory
    std::string fileDir = _modelPath + _modelName + "/";

    // only .obj is supported for now
    std::string extention = ".obj";

    std::string filePath = get_first_with_extention(fileDir, extention).string();

    Entity model = _active_scene->create_entity(_modelName);
    model.get_component<TransformComponent>().set_parent(parent);
    model.get_component<TransformComponent>().setScale(scaling);
    model.get_component<TransformComponent>().set_rotation(rotation);

    // Import aiScene
    Assimp::Importer import;
    const aiScene *scene = import.ReadFile(filePath, aiProcess_Triangulate | aiProcess_FlipUVs |
                                                         aiProcess_GenSmoothNormals | aiProcess_JoinIdenticalVertices);
    if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
        std::cout << "ERROR::ASSIMP::" << import.GetErrorString() << std::endl;
    } else {  // Process Nodes Recursive
        process_node(scene->mRootNode, scene, modelShader);
        model.add_component<RenderableElementComponent>(modelShader, _meshes);
    }
    return model;
}

void ModelLoader::process_node(aiNode *node, const aiScene *scene, const int shader) {
    // process all the node's meshes (if any)
    for (unsigned int i = 0; i < node->mNumMeshes; i++) {
        aiMesh *mesh = scene->mMeshes[node->mMeshes[i]];
        _meshes.push_back(process_mesh(mesh, scene, shader));
    }
    // then do the same for each of its children
    for (unsigned int i = 0; i < node->mNumChildren; i++) {
        process_node(node->mChildren[i], scene, shader);
    }
}

Mesh ModelLoader::process_mesh(aiMesh *mesh, const aiScene *scene, const int shader) {
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    std::vector<Texture> textures;
    aiColor4D *materialColor = new aiColor4D;
    aiMaterial *material = scene->mMaterials[mesh->mMaterialIndex];
    textures = load_material_textures(material, aiTextureType_DIFFUSE, shader);
    aiGetMaterialColor(material, AI_MATKEY_COLOR_DIFFUSE, materialColor);
    for (unsigned int i = 0; i < mesh->mNumVertices; i++) {
        Vertex vertex;
        vertex.position.x = mesh->mVertices[i].x;
        vertex.position.y = mesh->mVertices[i].y;
        vertex.position.z = mesh->mVertices[i].z;
        vertex.tex_coord = (mesh->mTextureCoords[0])
                              ? glm::vec2(mesh->mTextureCoords[0][i].x, mesh->mTextureCoords[0][i].y)
                              : glm::vec2(0.0f, 0.0f);
        vertex.tex_color = glm::vec3(materialColor->r, materialColor->g, materialColor->b);
        vertices.emplace_back(vertex);
    }
    for (unsigned int i = 0; i < mesh->mNumFaces; i++) {
        aiFace face = mesh->mFaces[i];
        for (unsigned int j = 0; j < face.mNumIndices; j++)
            indices.emplace_back(face.mIndices[j]);
    }
    return Mesh(vertices, indices, textures);
}

std::vector<Texture> ModelLoader::load_material_textures(aiMaterial *mat, aiTextureType type, const int shader) {
    std::vector<Texture> textures;
    unsigned int i = 0;
    do {
        aiString str;
        mat->GetTexture(type, i, &str);

        if (mat->GetTextureCount(type) == 0) {
            textures.emplace_back(_emptyTexture);
            return textures;
        }

        bool skip = false;
        for (unsigned int j = 0; j < _textures.size(); j++) {
            if (std::strcmp(_texturePaths.at(j).data(), str.C_Str()) == 0) {
                textures.emplace_back(Texture());
                textures.back().id = _textures[j].id;
                textures.back().type = GL_TEXTURE_2D;
                skip = true;
                break;
            }
        }
        if (!skip) {  // if texture hasn't been loaded already, load it
            textures.emplace_back(texture_from_file(str.C_Str(), shader));
            _texturePaths.emplace_back(std::string(str.C_Str()));
            _textures.emplace_back(textures.back());  // add to loaded textures
        }
        i++;
    } while (i < mat->GetTextureCount(type));
    return textures;
}

boost::filesystem::path ModelLoader::get_first_with_extention(const boost::filesystem::path &root,
                                                           const std::string &extention) {
    if (boost::filesystem::exists(root) && boost::filesystem::is_directory(root)) {
        for (auto const &entry : fs::recursive_directory_iterator(root)) {
            if (boost::filesystem::is_regular_file(entry) && entry.path().extension() == extention)
                return entry.path();
        }
    }
    return {};
}
Texture ModelLoader::texture_from_file(const char *textureName, const int shader) {
    std::string filename = std::string(textureName);
    filename = _modelPath + _modelName + "/" + filename;
    int width, height, nrComponents;
    unsigned char *data = stbi_load(filename.c_str(), &width, &height, &nrComponents, 0);
    if (!data) {
        // TODO(Andi): port print
        // ROS_ERROR_STREAM("In ModelLoader::texture_from_file(): Texture failed to load at path: "
        //     << filename);
        return Texture();
    }
    GLenum format{GL_NONE};
    if (nrComponents == 1)
        format = GL_RED;
    else if (nrComponents == 3)
        format = GL_RGB;
    else if (nrComponents == 4)
        format = GL_RGBA;
    else {
        // TODO(Andi): port print
        // ROS_WARN("In ModelLoader::texture_from_file(): format not initialized");
    }
    Texture texture(width, height, "texture_diffuse", GL_TEXTURE_2D, format, format);
    Renderer::generate_texture(texture, data, shader, 0);
    stbi_image_free(data);
    return texture;
}

};  // namespace tod_gl