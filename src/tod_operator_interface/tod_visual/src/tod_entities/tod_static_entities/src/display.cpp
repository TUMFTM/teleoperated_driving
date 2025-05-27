/**
 * @file display.cpp
 * @brief Display is a 3D rendered Car HUD with text 
 * @copyright 2024 TUMFTM
 **/

#include "tod_static_entities/display.hpp"

#include "tod_gl/renderer/renderer_command.hpp"

namespace TodStaticEntities {

tod_gl::Entity Display::create(std::shared_ptr<tod_gl::Scene> scene, std::string name, tod_gl::Entity parent,
                               const std::string& packagePath) {
    tod_gl::Entity display = scene->create_entity(name);
    auto& charMap = display.add_component<tod_gl::CharacterMapComponent>();
    display.get_component<tod_gl::TransformComponent>().set_parent(parent);

    unsigned int tachoShader =
        tod_gl::ShaderSystem::create_shader_program((packagePath + "/resources/shaders/character.vert").c_str(),
                                                  (packagePath + "/resources/shaders/character.frag").c_str());

    unsigned int numberOfLetters{5};
    unsigned int numberOfVerticesPerLetter{4};
    std::vector<tod_gl::Mesh> meshes;
    for (unsigned int iterator = 0; iterator != numberOfLetters; ++iterator) {
        std::vector<tod_gl::Vertex> vertices(numberOfVerticesPerLetter);
        std::vector<unsigned int> indices{0, 1, 3, 1, 2, 3};
        std::vector<tod_gl::Texture> textures{tod_gl::Texture()};
        tod_gl::Mesh Mesh(vertices, indices, textures);
        meshes.push_back(Mesh);
    }
    auto& renderable = display.add_component<tod_gl::RenderableElementComponent>(tachoShader, meshes);
    display.add_component<tod_gl::DynamicDataComponent>();

    GenerateCharactersWithTextureForDisplay("/usr/share/fonts/truetype/liberation/LiberationMono-BoldItalic.ttf",
                                            charMap, renderable);

    return display;
}

void Display::onSpeedUpdate(const tod_vehicle_msgs::msg::PrimaryVehicleState::ConstSharedPtr& msg, tod_gl::Entity& entity) {
    auto& dynamic = entity.get_component<tod_gl::DynamicDataComponent>();
    auto& renderable = entity.get_component<tod_gl::RenderableElementComponent>();
    auto& characterMap = entity.get_component<tod_gl::CharacterMapComponent>();
    std::lock_guard<std::mutex> lock(*dynamic.mutex);
    dynamic.has_new_data = true;
    clearVerticesAndTextures(renderable);
    float istGeschwindigkeitFloat = 3.6f * msg->velocity;
    std::string istGeschwindigkeit = floatToIntString(istGeschwindigkeitFloat);
    writeTextIntoRenderable(istGeschwindigkeit, renderable, characterMap);
}

void Display::onDesiredSpeedUpdate(const tod_vehicle_msgs::msg::PrimaryControlCmd::ConstSharedPtr& msg,
                                   tod_gl::Entity& entity) {
    auto& dynamic = entity.get_component<tod_gl::DynamicDataComponent>();
    auto& renderable = entity.get_component<tod_gl::RenderableElementComponent>();
    auto& characterMap = entity.get_component<tod_gl::CharacterMapComponent>();
    std::lock_guard<std::mutex> lock(*dynamic.mutex);
    dynamic.has_new_data = true;
    clearVerticesAndTextures(renderable);
    float sollGeschwindigkeitFloat = 3.6f * msg->velocity;
    std::string sollGeschwindigkeit = floatToIntString(sollGeschwindigkeitFloat);
    sollGeschwindigkeit += " /";
    writeTextIntoRenderable(sollGeschwindigkeit, renderable, characterMap);
}

void Display::onGearUpdate(const tod_vehicle_msgs::msg::SecondaryVehicleState::ConstSharedPtr& msg, tod_gl::Entity& entity) {
    auto& dynamic = entity.get_component<tod_gl::DynamicDataComponent>();
    auto& renderable = entity.get_component<tod_gl::RenderableElementComponent>();
    auto& characterMap = entity.get_component<tod_gl::CharacterMapComponent>();
    std::lock_guard<std::mutex> lock(*dynamic.mutex);
    dynamic.has_new_data = true;
    clearVerticesAndTextures(renderable);
    std::string gearPosition = eGearPositionToString(msg->gear_position);
    writeTextIntoRenderable(gearPosition, renderable, characterMap);
}

void Display::onDesiredGearUpdate(const tod_vehicle_msgs::msg::SecondaryControlCmd::ConstSharedPtr& msg,
                                  tod_gl::Entity& entity) {
    auto& dynamic = entity.get_component<tod_gl::DynamicDataComponent>();
    auto& renderable = entity.get_component<tod_gl::RenderableElementComponent>();
    auto& characterMap = entity.get_component<tod_gl::CharacterMapComponent>();
    std::lock_guard<std::mutex> lock(*dynamic.mutex);
    dynamic.has_new_data = true;
    clearVerticesAndTextures(renderable);
    std::string desiredGearPosition = eGearPositionToString(msg->gear_position);
    desiredGearPosition += "/";
    writeTextIntoRenderable(desiredGearPosition, renderable, characterMap);
}

void Display::GenerateCharactersWithTextureForDisplay(const std::string& pathToTTF,
                                                      tod_gl::CharacterMapComponent& charMap,
                                                      tod_gl::RenderableElementComponent& renderable) {
    charMap.characters.clear();

    FT_Library ft;
    if (FT_Init_FreeType(&ft)) {
        std::cerr << "tod_gl - ERROR::FREETYPE: Could not init FreeType Library" << std::endl;
        return;
    }

    std::string font_name = pathToTTF;
    if (font_name.empty()) {
        std::cerr << "tod_gl - ERROR::FREETYPE: Failed to load font_name" << std::endl;
        return;
    }

    FT_Face face;
    if (FT_New_Face(ft, font_name.c_str(), 0, &face)) {
        std::cerr << "tod_gl - ERROR::FREETYPE: Failed to load font from path " << font_name << std::endl;
        return;
    }
    FT_Set_Pixel_Sizes(face, 0, (FT_UInt)charMap.pixel_per_meter_ratio);  // height big enough not to be pixelated

    // disable byte-alignment restriction
    tod_gl::RenderCommand::set_pixel_storage_mode(GL_UNPACK_ALIGNMENT, 1);

    // load first 128 characters of ASCII set
    for (unsigned char c = 0; c < 128; c++) {
        if (FT_Load_Char(face, c, FT_LOAD_RENDER)) {
            std::cerr << "tod_gl - ERROR::FREETYTPE: Failed to load Glyph" << std::endl;
            continue;
        }
        // generate texture and store in character for later use
        tod_gl::Texture texture;
        texture.name = "text";
        texture.type = GL_TEXTURE_2D;
        texture.width = face->glyph->bitmap.width;
        texture.height = face->glyph->bitmap.rows;
        tod_gl::Renderer::generate_texture(texture, face->glyph->bitmap.buffer, renderable.shader_program, 0);
        tod_gl::Character character = {texture, glm::ivec2(face->glyph->bitmap.width, face->glyph->bitmap.rows),
                                       glm::ivec2(face->glyph->bitmap_left, face->glyph->bitmap_top),
                                       static_cast<unsigned int>(face->glyph->advance.x)};
        charMap.characters.insert(std::pair<char, tod_gl::Character>(c, character));
    }

    FT_Done_Face(face);
    FT_Done_FreeType(ft);
}

void Display::clearVerticesAndTextures(tod_gl::RenderableElementComponent& renderable) {
    for (auto& Mesh : renderable.meshes) {
        Mesh.vertices.clear();
        Mesh.textures.clear();
    }
}

void Display::writeTextIntoRenderable(const std::string& text, tod_gl::RenderableElementComponent& renderable,
                                      const tod_gl::CharacterMapComponent& characterMap) {
    float tmpAdvance{0.0f};
    float scale{0.15f};
    float y{0.05f};
    for (unsigned int index = 0; index != text.size(); ++index) {
        if (index >= renderable.meshes.size()) {
            std::cerr << "More letters than meshes" << std::endl;
            continue;
        }
        auto& currentMesh = renderable.meshes.at(index);
        tod_gl::Character currentChar = characterMap.characters.at(text.at(index));
        currentMesh.textures.emplace_back(currentChar.tex);
        updateVertices(currentMesh, tmpAdvance, currentChar, scale, y, characterMap.pixel_per_meter_ratio);
        updateAdvance(tmpAdvance, currentChar, scale);
    }
}

std::string Display::floatToIntString(const float number) {
    int numberOfDigits{4};
    if (std::abs(number) >= 1000.0f) {
        printf("In Display::floatToIntString(): Float to large\n");
    }
    char snumber[numberOfDigits];
    size_t sizet{(size_t)numberOfDigits + 1};
    strfromf(&snumber[0], sizet, "%.0f", number);
    std::string tempString{snumber};
    return tempString;
}

std::string Display::eGearPositionToString(const int gearPosition) {
    std::string gearPositionString;
    switch ((eGearPosition)gearPosition) {
        case eGearPosition::GEARPOSITION_PARK:
            gearPositionString = "P";
            break;
        case eGearPosition::GEARPOSITION_REVERSE:
            gearPositionString = "R";
            break;
        case eGearPosition::GEARPOSITION_NEUTRAL:
            gearPositionString = "N";
            break;
        case eGearPosition::GEARPOSITION_DRIVE:
            gearPositionString = "D";
            break;
        default:
            gearPositionString = "P";
            break;
    }
    return gearPositionString;
}

void Display::updateVertices(tod_gl::Mesh& currentMesh, const float tmpAdvance, const tod_gl::Character& currentChar,
                             const float& scale, const float& y, const float& ratioPixelPerMeter) {
    float xpos = 1.0f / ratioPixelPerMeter * (tmpAdvance + currentChar.bearing.x * scale);
    float ypos = 1.0f / ratioPixelPerMeter * (y - (currentChar.size.y - currentChar.bearing.y) * scale);
    float w = 1.0f / ratioPixelPerMeter * (currentChar.size.x * scale);
    float h = 1.0f / ratioPixelPerMeter * (currentChar.size.y * scale);
    currentMesh.vertices.emplace_back(glm::vec3(0.0f, -xpos, ypos + h), glm::vec2(0.0f, 0.0f), glm::vec3());
    currentMesh.vertices.emplace_back(glm::vec3(0.0f, -xpos, ypos), glm::vec2(0.0f, 1.0f), glm::vec3());
    currentMesh.vertices.emplace_back(glm::vec3(0.0f, -(xpos + w), ypos), glm::vec2(1.0f, 1.0f), glm::vec3());
    currentMesh.vertices.emplace_back(glm::vec3(0.0f, -(xpos + w), ypos + h), glm::vec2(1.0f, 0.0f), glm::vec3());
}

void Display::updateAdvance(float& tmpAdvance, const tod_gl::Character& currentChar, const float& scale) {
    // bitshift by 6 to get value in pixels
    // (2^6 = 64 (divide amount of 1/64th pixels by 64 to get amount of pixels))
    tmpAdvance += (currentChar.advance >> 6) * scale;
}

};  // namespace TodStaticEntities
