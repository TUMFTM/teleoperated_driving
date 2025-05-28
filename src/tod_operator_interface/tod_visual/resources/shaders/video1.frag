#version 330 core
// pointer to texture buffers for Y, U and V values
uniform sampler2DRect Ytex;
uniform sampler2DRect Utex;
uniform sampler2DRect Vtex;

out vec4 colour;
in vec2 TexCoord;

mediump float gamma = 0.5;
mediump float Contrast = 1.35;

void main(void) {

    float y, u, v, r, g, b, nx, ny; // , rTmp, gTmp, bTmp;

    y = texture2DRect(Ytex, vec2(TexCoord.x, TexCoord.y)).r;
    u = texture2DRect(Utex, vec2(TexCoord.x / 2, TexCoord.y / 4)).r;
    v = texture2DRect(Vtex, vec2(TexCoord.x / 2, TexCoord.y / 4)).r;

    y = 1.1643 * (y - 0.0625);
    u = u - 0.5;
    v = v - 0.5;

    r = clamp(y + 1.5958 * v, 0, 1);
    g = clamp(y - 0.39173 * u - 0.81290 * v, 0, 1);
    b = clamp(y + 2.017 * u, 0, 1);

    // gamma correction
    r = pow(r, 1.0 / gamma);
    g = pow(g, 1.0 / gamma);
    b = pow(b, 1.0 / gamma);

    r = clamp(((r - 0.5f) * max(Contrast, 0)) + 0.5f, 0, 1);
    g = clamp(((g - 0.5f) * max(Contrast, 0)) + 0.5f, 0, 1);
    b = clamp(((b - 0.5f) * max(Contrast, 0)) + 0.5f, 0, 1);

    colour = vec4(r, g, b, 1);
}

//  } else {
//    if (x_distance < y_distance) {
//      colour = vec4(
//          r, g, b, 1.0); // was opac before
//    } else {
//      colour = vec4(r, g, b, 1.0);
//    }
//  }

// colour=vec4(v,v,v,1.0f);
