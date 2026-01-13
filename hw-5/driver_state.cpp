#include "driver_state.h"
#include <cstring>

driver_state::driver_state()
{
}

driver_state::~driver_state()
{
    delete [] image_color;
    delete [] image_depth;
}

// This function should allocate and initialize the arrays that store color and
// depth.  This is not done during the constructor since the width and height
// are not known when this class is constructed.
void initialize_render(driver_state& state, int width, int height)
{
    state.image_width=width;
    state.image_height=height;
    state.image_color=0;
    state.image_depth=0;
    //std::cout<<"TODO: allocate and initialize state.image_color and state.image_depth."<<std::endl;
    state.image_color = new pixel[width*height];
    state.image_depth = new float[width*height];
    for(int i = 0; i < width * height; i++) {
        state.image_color[i] = make_pixel(0, 0, 0);
        state.image_depth[i] = 1.0f; // Initialize depth to farthest
    }
    

}

// This function will be called to render the data that has been stored in this class.
// Valid values of type are:
//   render_type::triangle - Each group of three vertices corresponds to a triangle.
//   render_type::indexed -  Each group of three indices in index_data corresponds
//                           to a triangle.  These numbers are indices into vertex_data.
//   render_type::fan -      The vertices are to be interpreted as a triangle fan.
//   render_type::strip -    The vertices are to be interpreted as a triangle strip.
void render(driver_state& state, render_type type)
{
    //std::cout<<"TODO: implement rendering."<<std::endl;
    if(type == render_type::triangle){
        int numTriangles = state.num_vertices / 3;

        for(int i = 0; i < numTriangles; i++){
            data_geometry geo[3];
            for(int j = 0; j < 3; j++){
                geo[j].data = new float[MAX_FLOATS_PER_VERTEX];

                data_vertex vert;
                vert.data = &state.vertex_data[(i * 3 + j) * state.floats_per_vertex];
                state.vertex_shader(vert, geo[j], state.uniform_data);
            }
            clip_triangle(state, geo[0], geo[1], geo[2]);
            for(int j = 0; j < 3; j++){
                delete[] geo[j].data;
            }

        }

    }
    if(type == render_type::indexed){
        for(int i = 0; i < state.num_triangles; i++){
            data_geometry geo[3];
            for(int j = 0; j < 3; j++){
                geo[j].data = new float[MAX_FLOATS_PER_VERTEX];

                int index = state.index_data[i * 3 + j];
                data_vertex vert;
                vert.data = &state.vertex_data[index * state.floats_per_vertex];
                state.vertex_shader(vert, geo[j], state.uniform_data);
            }
            clip_triangle(state, geo[0], geo[1], geo[2]);
            for(int j = 0; j < 3; j++){
                delete[] geo[j].data;
            }
        }
    }
    if(type == render_type::fan){
        if(state.num_vertices < 3) return;
        
        data_geometry geo0;
        geo0.data = new float[MAX_FLOATS_PER_VERTEX];
        data_vertex vert0;
        vert0.data = &state.vertex_data[0];
        state.vertex_shader(vert0, geo0, state.uniform_data);
        
        for(int i = 1; i < state.num_vertices - 1; i++){
            data_geometry geo1, geo2;
            geo1.data = new float[MAX_FLOATS_PER_VERTEX];
            geo2.data = new float[MAX_FLOATS_PER_VERTEX];
            
            data_vertex vert1, vert2;
            vert1.data = &state.vertex_data[i * state.floats_per_vertex];
            vert2.data = &state.vertex_data[(i + 1) * state.floats_per_vertex];
            
            state.vertex_shader(vert1, geo1, state.uniform_data);
            state.vertex_shader(vert2, geo2, state.uniform_data);
            
            clip_triangle(state, geo0, geo1, geo2);
            
            delete[] geo1.data;
            delete[] geo2.data;
        }
        
        delete[] geo0.data;
    }
   if(type == render_type::strip){
        if(state.num_vertices < 3) return;
        
        for(int i = 0; i < state.num_vertices - 2; i++){
            data_geometry geo[3];
            for(int j = 0; j < 3; j++){
                geo[j].data = new float[MAX_FLOATS_PER_VERTEX];
            }
            
            data_vertex vert0, vert1, vert2;
            
            if(i % 2 == 0){
                vert0.data = &state.vertex_data[i * state.floats_per_vertex];
                vert1.data = &state.vertex_data[(i + 1) * state.floats_per_vertex];
                vert2.data = &state.vertex_data[(i + 2) * state.floats_per_vertex];
            } 
            else {
                vert0.data = &state.vertex_data[(i + 1) * state.floats_per_vertex];
                vert1.data = &state.vertex_data[i * state.floats_per_vertex];
                vert2.data = &state.vertex_data[(i + 2) * state.floats_per_vertex];
            }
            
            state.vertex_shader(vert0, geo[0], state.uniform_data);
            state.vertex_shader(vert1, geo[1], state.uniform_data);
            state.vertex_shader(vert2, geo[2], state.uniform_data);
            
            clip_triangle(state, geo[0], geo[1], geo[2]);
            
            for(int j = 0; j < 3; j++){
                delete[] geo[j].data;
            }
        }
    }
    
}

int plane(int plane, const data_geometry* in[], int numIn, const data_geometry* out[], data_geometry* allocated[], int& allocCount,
                              int fpv)
{
    if(numIn == 0) return 0;
    int numOut = 0;
    
    // if 
    auto inside = [plane](const vec4& p) -> bool {
        if(plane == 0) return p[2] + p[3] >= 0;       
        else if(plane == 1) return p[3] - p[2] >= 0;  
        else return p[3] > 0.001f;                    
    };
    //if z + w >= 0
    auto signedDistance = [plane](const vec4& p) -> float {
        if(plane == 0) return p[2] + p[3];      
        else if(plane == 1) return p[3] - p[2]; 
        else return p[3] - 0.001f;              
    };
    
    for(int i = 0; i < numIn; i++)
    {
        int j = (i + 1) % numIn;
        const data_geometry* v1 = in[i];
        const data_geometry* v2 = in[j];
        
        bool v1Inside = inside(v1->gl_Position);
        bool v2Inside = inside(v2->gl_Position);
        
        if(v1Inside)
        {
            out[numOut++] = v1;
        }
        
        if(v1Inside != v2Inside)
        {
            float d1 = signedDistance(v1->gl_Position);
            float d2 = signedDistance(v2->gl_Position);
            float t = d1 / (d1 - d2);
            
            data_geometry* clipped = new data_geometry;
            clipped->data = new float[MAX_FLOATS_PER_VERTEX];
            
            clipped->gl_Position = v1->gl_Position + (v2->gl_Position - v1->gl_Position) * t;
            for(int k = 0; k < fpv; k++)
            {
                clipped->data[k] = v1->data[k] + t * (v2->data[k] - v1->data[k]);
            }
            
            allocated[allocCount++] = clipped;
            out[numOut++] = clipped;
        }
    }
    
    return numOut;
}

void clip_triangle(driver_state& state, const data_geometry& v0,
    const data_geometry& v1, const data_geometry& v2)
{
   
    const data_geometry* input[8];
    const data_geometry* output[8];
    data_geometry* allocated[32];
    int allocCount = 0;
    
    input[0] = &v0;
    input[1] = &v1;
    input[2] = &v2;
    int numVerts = 3;
    
    
    numVerts = plane(2, input, numVerts, output, allocated, allocCount, state.floats_per_vertex);
    
    if(numVerts < 3)
    {
        for(int i = 0; i < allocCount; i++)
        {
            delete[] allocated[i]->data;
            delete allocated[i];
        }
        return;
    }
    
    for(int i = 0; i < numVerts; i++)
        input[i] = output[i];
    
    numVerts = plane(0, input, numVerts, output, allocated, allocCount, state.floats_per_vertex);
    
    if(numVerts < 3)
    {
        for(int i = 0; i < allocCount; i++)
        {
            delete[] allocated[i]->data;
            delete allocated[i];
        }
        return;
    }
    
    for(int i = 0; i < numVerts; i++)
        input[i] = output[i];
    
    numVerts = plane(1, input, numVerts, output, allocated, allocCount, state.floats_per_vertex);
    
    if(numVerts >= 3){
        for(int i = 1; i < numVerts - 1; i++)
        {
            rasterize_triangle(state, *output[0], *output[i], *output[i+1]);
        }
    }
    
    for(int i = 0; i < allocCount; i++)
    {
        delete[] allocated[i]->data;
        delete allocated[i];
    }
}

// Rasterize the triangle defined by the three vertices.  This routine should
// pass triangles on to clip_triangle for clipping and rendering.
void rasterize_triangle(driver_state& state, const data_geometry& v0,
    const data_geometry& v1, const data_geometry& v2)
{
    //std::cout<<"TODO: implement rasterization"<<std::endl;
    float w0 = v0.gl_Position[3];
    float w1 = v1.gl_Position[3];
    float w2 = v2.gl_Position[3];
    
    if(w0 == 0.0f) w0 = 1.0f;
    if(w1 == 0.0f) w1 = 1.0f;
    if(w2 == 0.0f) w2 = 1.0f;
    
    float ndcX0 = v0.gl_Position[0] / w0;
    float ndcY0 = v0.gl_Position[1] / w0;
    float ndcZ0 = v0.gl_Position[2] / w0;
    
    float ndcX1 = v1.gl_Position[0] / w1;
    float ndcY1 = v1.gl_Position[1] / w1;
    float ndcZ1 = v1.gl_Position[2] / w1;
    
    float ndcX2 = v2.gl_Position[0] / w2;
    float ndcY2 = v2.gl_Position[1] / w2;
    float ndcZ2 = v2.gl_Position[2] / w2;
    
    float x0 = (ndcX0 + 1.0f) * state.image_width / 2.0f;
    float y0 = (ndcY0 + 1.0f) * state.image_height / 2.0f;
    
    float x1 = (ndcX1 + 1.0f) * state.image_width / 2.0f;
    float y1 = (ndcY1 + 1.0f) * state.image_height / 2.0f;
    
    float x2 = (ndcX2 + 1.0f) * state.image_width / 2.0f;
    float y2 = (ndcY2 + 1.0f) * state.image_height / 2.0f;
    
    int minX = std::max(0, (int)std::floor(std::min(std::min(x0, x1), x2)));
    int maxX = std::min(state.image_width - 1, (int)std::ceil(std::max(std::max(x0, x1), x2)));
    int minY = std::max(0, (int)std::floor(std::min(std::min(y0, y1), y2)));
    int maxY = std::min(state.image_height - 1, (int)std::ceil(std::max(std::max(y0, y1), y2)));
    
    float area = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0);
    
    if(std::abs(area) < 0.0001f) return;
    for(int y = minY; y <= maxY; y++)
    {
        for(int x = minX; x <= maxX; x++)
        {
            float px = x + 0.5f;
            float py = y + 0.5f;
            
            float alpha = ((x1 - px) * (y2 - py) - (x2 - px) * (y1 - py)) / area;
            float beta = ((x2 - px) * (y0 - py) - (x0 - px) * (y2 - py)) / area;
            float gamma = 1.0f - alpha - beta;
            
            if(alpha >= 0 && beta >= 0 && gamma >= 0)
            {
                float depth = alpha * ndcZ0 + beta * ndcZ1 + gamma * ndcZ2;
                
                int pixelInd = y * state.image_width + x;
                
                if(depth < state.image_depth[pixelInd])
                {
                    state.image_depth[pixelInd] = depth;
                    
                    data_fragment fragment;
                    fragment.data = new float[MAX_FLOATS_PER_VERTEX];
                    
                    for(int i = 0; i < state.floats_per_vertex; i++)
                    {
                        switch(state.interp_rules[i])
                        {
                            case interp_type::flat:
                                fragment.data[i] = v0.data[i];
                                break;
                                
                            case interp_type::smooth:
                                {
                                    float interp_w0 = alpha / w0;
                                    float interp_w1 = beta / w1;
                                    float interp_w2 = gamma / w2;
                                    float w_sum = interp_w0 + interp_w1 + interp_w2;
                                    fragment.data[i] = (interp_w0 * v0.data[i] + 
                                                       interp_w1 * v1.data[i] + 
                                                       interp_w2 * v2.data[i]) / w_sum;
                                }
                                break;
                                
                            case interp_type::noperspective:
                                fragment.data[i] = alpha * v0.data[i] + beta * v1.data[i] + gamma * v2.data[i];
                                break;
                                
                            default:
                                fragment.data[i] = alpha * v0.data[i] + beta * v1.data[i] + gamma * v2.data[i];
                                break;
                        }
                    }
                    
                    data_output output;
                    state.fragment_shader(fragment, output, state.uniform_data);
                    
                    int r = std::min(255, std::max(0, (int)(output.output_color[0] * 255)));
                    int g = std::min(255, std::max(0, (int)(output.output_color[1] * 255)));
                    int b = std::min(255, std::max(0, (int)(output.output_color[2] * 255)));
                    
                    state.image_color[pixelInd] = make_pixel(r, g, b);
                    
                    delete [] fragment.data;
                }
            }
        }
    }
}

