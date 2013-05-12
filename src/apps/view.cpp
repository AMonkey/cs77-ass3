#include "igl/scene.h"
#include "igl/serialize.h"
#include "igl/gls.h"
#include "igl/gl_utils.h"
#include "common/common.h"
#include "tclap/CmdLine.h"

#include "igl/draw.h"
#include "igl/intersect.h"
#include "igl/tesselate.h"

#define AUTORELOAD

#ifdef AUTORELOAD
#include <sys/stat.h>
#endif

///@file apps/view.cpp View: Interactice Viewer @ingroup apps
///@defgroup view View: Interactice Viewer
///@ingroup apps
///@{

const bool          fixed_dt = true; ///< fixed frame time

bool                screenshotAndExit = false; ///< take a screenshot and exit right away

Scene*              scene = nullptr; ///< scene

DrawOptions         draw_opts; ///< draw options

float               time_init_advance = 0.0f; ///< how much to initially advance time
bool                animating = false; ///< whether it is currently playing back animation
bool                animating_steps = false; 
range1f             animate_interval = range1f(); ///< scene animation interval
bool                animate_loop = false; ///< whether to loop the animation
bool                simulate_has = false; ///< whether the scene has simulation

string              filename_scene = ""; ///< scene filename
string              filename_image = ""; ///< captured image filename

int                 tesselation_level = -1; ///< tesselation override level (-1 for default)
bool                tesselation_smooth = false; ///< tesselation override smooth

bool                hud = true; ///< whether to display the hud
timer_avg           hud_fps_update; ///< whether to display update frames-per-second in the hud
timer_avg           hud_fps_display; ///< whether to display draw frames-per-second in the hud

int                 selected_element = -1; ///< selected scene element (-1 for no selection)
int                 selected_subelement = -1; ///< selected scene subelement (-1 for no selection)
frame3f*            selected_frame = nullptr; ///< selected frame pointer (used to edit)
vec3f*              selected_point = nullptr; ///< selected point pointer (used to edit)

float               selecion_speed_keyboard_translate = 0.5f; ///< selection speed
float               selecion_speed_keyboard_rotate = pif / 8; ///< selection speed
float               selecion_speed_mouse_translate = 1.0f / 100; ///< selection speed
float               selecion_speed_mouse_rotate = 1.0 / (pif / 8); ///< selection speed

/// update timer
auto update_timer = timer();

// start animation playback
void animate_start() { update_timer.start(); animating = true; }

/// stop animation playback
void animate_stop() { animating = false; }

/// update the scene during animation playback
void animate_update() {
    float dt = update_timer.elapsed();
    if(fixed_dt) dt = clamp(1/30.0f,0.0f,animate_interval.max-draw_opts.time);
    else dt = clamp(dt,0.0f,animate_interval.max-draw_opts.time);
    if(dt > 0) {
        draw_opts.time += dt;
        scene_simulation_update(scene,dt);
        update_timer.start();
    } else {
        if(draw_opts.time >= animate_interval.max) animate_stop();
    }

    if(time_init_advance > 0.0f and draw_opts.time >= time_init_advance) {
        animate_stop();
        time_init_advance = 0.0f;
    }
}

/// update the scene for one step in animation playback
void animate_step(bool forward) {
    float dt;
    if(forward) dt = clamp(1/30.0f,0.0f,animate_interval.max-draw_opts.time);
    else dt = clamp(-1/30.0f,animate_interval.min-draw_opts.time,0.0f);
    if(dt != 0) {
        draw_opts.time += dt;
        if(simulate_has) scene_simulation_update(scene,dt);
    }
}

/// clear subelement selection
void selection_subelement_clear() {
    selected_subelement = -1;
    selected_point = nullptr;
}

/// clear element selection
void selection_element_clear() {
    selected_element = -1;
    selected_frame = nullptr;
    selection_subelement_clear();
}

/// select next element
void selection_element_next(bool forward) {
    auto nprims = scene->prims->prims.size();
    auto nlights = scene->lights->lights.size();
    auto nelements = nprims + nlights;
    if(selected_element < 0) selected_element = (forward) ? 0 : nelements-1;
    else selected_element = (selected_element + ( (forward) ? 1 : -1 ) + nelements) % nelements;
    selected_frame = (selected_element < nprims) ? &scene->prims->prims[selected_element]->frame : &scene->lights->lights[selected_element - nprims]->frame;
    selection_subelement_clear();
}

/// select next subelement
void selection_subelement_next(bool forward) {
    if(selected_element >= scene->prims->prims.size() or selected_element < 0) return;
    auto prim = scene->prims->prims[selected_element];
    if(not is<Surface>(prim)) return;
    auto shape = cast<Surface>(prim)->shape;
    auto pos = shape_get_pos(shape);
    if(not pos) return;
    auto nelements = pos->size();
    if(selected_subelement < 0) selected_subelement = (forward) ? 0 : nelements-1;
    else selected_subelement = (selected_subelement + ( (forward) ? 1 : -1 ) + nelements) % nelements;
    selected_point = &pos->at(selected_subelement);
}

/// move selection
void selection_move(const vec3f& t) {
    if(selected_point) {
        *selected_point += transform_vector(*selected_frame,t);
        shape_tesselation_init(cast<Surface>(scene->prims->prims[selected_element])->shape, tesselation_level >= 0, tesselation_level, tesselation_smooth);
    }
    else if(selected_frame) selected_frame->o += transform_vector(*selected_frame,t);
}

/// rotate selection
void selection_rotate(const vec3f& ea) {
    if(selected_point) return;
    if(selected_frame) {
        auto m = translation_matrix(selected_frame->o) *
                 rotation_matrix(ea.x, selected_frame->x) *
                 rotation_matrix(ea.y, selected_frame->y) *
                 rotation_matrix(ea.z, selected_frame->z) *
                 translation_matrix(- selected_frame->o);
        *selected_frame = transform_frame(m, *selected_frame);
    }
}

/// init scene
void init() {
    scene_tesselation_init(scene,tesselation_level>=0,tesselation_level,tesselation_smooth);
    scene_defaultgizmos_init(scene);
    animate_interval = scene_animation_interval(scene);
    simulate_has = scene_simulation_has(scene);
    if(simulate_has) {
        animate_interval.max = 1e10f;
        scene_simulation_init(scene);
    }
    //draw_opts.time = 0;
    selection_element_clear();
}

/// grab pixels from the screen
image3f readpixels() {
    int w = camera_image_width(scene->camera,draw_opts.res);
    int h = camera_image_height(scene->camera,draw_opts.res);
    return glutils_read_pixels(w,h,true);
}

/// grab a screenshot and save it
void screenshot(const char *filename_png) {
    image3f img = readpixels();
    imageio_write_png(filename_png, img, true);
}

#ifdef AUTORELOAD
bool reload_auto = false;
#endif

/// load scene
void load() {
    scene = nullptr;
    Serializer::read_json(scene, filename_scene);
    if(not scene) { error("could not load scene"); }
    init();
}
/// reload the scene
void reload() {
    Scene* new_scene = nullptr;
    Serializer::read_json(new_scene, filename_scene);
    if(new_scene) {
        scene = new_scene;
        init();
    } else warning("could not reload scene");
}

/// change the view to frame the scene
void frame() {
    auto bbox = intersect_scene_bounds(scene);
    vec3f c = center(bbox);
    vec3f s = size(bbox);
    camera_view_lookat(scene->camera,c+scene->camera->frame.z*length(s), c, scene->camera->frame.y);
}

/// print commands
void print_help() {
    printf("viewing ----------------------------\n"
           "mouse-left          rotate\n"
           "mouse-right         dolly\n"
           "mouse-middle        pan\n"
           "mouse-ctrl          pan\n"
           "1                   faces on/off\n"
           "2                   edges on/off\n"
           "3                   lines on/off\n"
           "4                   control on/off\n"
           "$                   depth check for control on/off\n"
           "5                   gizmos on/off\n"
           "7                   camera lights on/off\n"
           "8                   scene tesselation descrease\n"
           "9                   scene tesselation increase\n"
           "0                   scene tesselation smooth\n"
           "`                   doublesided on/off\n"
           "f                   frame\n"
           "\n"
           "application ------------------------\n"
           "esc                 quit\n"
           "h                   print help\n"
           "r                   reload\n"
           "i                   save screenshot\n"
           "j                   hud on/off\n"
           "/                   reset\n"
           "\n"
           "animation --------------------------\n"
           "space               animation on/off\n"
           ",/.                 animation step\n"
           "\n"
           "editing ----------------------------\n"
           "[/]                 select primitive/light\n"
           "{/}                 select shape point\n"
           "\\/|                clear select primitive/point\n"
           "alt-mouse-left      translate x\n"
           "alt-mouse-right     translate y\n"
           "alt-mouse-middle    translate z\n"
           "ctrl-alt-mouse      translate z\n"
           "w/s a/d q/e         translate xyz\n"
           "shift               rotate xyz\n"
           "\n");
}

/// keyboard event handler
void keyboard(unsigned char c, int x, int y) {
	switch(c) {
        case 27: exit(0); break;
        case 'h': print_help(); break;
        case '1': draw_opts.faces = not draw_opts.faces; break;
        case '2': draw_opts.edges = not draw_opts.edges; break;
        case '3': draw_opts.lines = not draw_opts.lines; break;
        case '4': draw_opts.control = not draw_opts.control; break;
        case '$': draw_opts.control_no_depth = not draw_opts.control_no_depth; break;
        case '5': draw_opts.gizmos = not draw_opts.gizmos; break;
        case '7': draw_opts.cameralights = not draw_opts.cameralights; break;
        case '8': tesselation_level = max(-1,tesselation_level-1); init(); break;
        case '9': tesselation_level = min(88,tesselation_level+1); init(); break;
        case '0': tesselation_smooth = !tesselation_smooth; init(); break;
        case '`': draw_opts.doublesided = not draw_opts.doublesided; break;
        case ' ': if(animating) animate_stop(); else animate_start(); break;
        case '/': init(); break;
        case '?': animate_loop = !animate_loop; break;
        case ',': animate_step(false); break;
        case '.': animate_step(true); break;
        case 'j': hud = not hud; break;
        case 'f': frame(); break;
        case 'r': reload(); break;
        case 'i': screenshot(filename_image.c_str()); break;
#ifdef AUTORELOAD
        case 'R': reload_auto = not reload_auto; break;
#endif
        case '[': selection_element_next(false); break;
        case ']': selection_element_next(true); break;
        case '\\': selection_element_clear(); break;
        case '{': selection_subelement_next(false); break;
        case '}': selection_subelement_next(true); break;
        case '|': selection_subelement_clear(); break;
        case 'a': selection_move( - x3f * selecion_speed_keyboard_translate ); break;
        case 'd': selection_move(   x3f * selecion_speed_keyboard_translate ); break;
        case 'w': selection_move(   z3f * selecion_speed_keyboard_translate ); break;
        case 's': selection_move( - z3f * selecion_speed_keyboard_translate ); break;
        case 'q': selection_move( - y3f * selecion_speed_keyboard_translate ); break;
        case 'e': selection_move(   y3f * selecion_speed_keyboard_translate ); break;
        case 'A': selection_rotate( - x3f * selecion_speed_keyboard_rotate ); break;
        case 'D': selection_rotate(   x3f * selecion_speed_keyboard_rotate ); break;
        case 'W': selection_rotate(   z3f * selecion_speed_keyboard_rotate ); break;
        case 'S': selection_rotate( - z3f * selecion_speed_keyboard_rotate ); break;
        case 'Q': selection_rotate( - y3f * selecion_speed_keyboard_rotate ); break;
        case 'E': selection_rotate(   y3f * selecion_speed_keyboard_rotate ); break;
		default: break;
	}
    
	glutPostRedisplay();
}

/// mouse action types
enum MouseAction {
    mouse_none,
    mouse_turntable_rotate,
    mouse_turntable_dolly,
    mouse_turntable_pan,
    mouse_edit_frame_move_x,
    mouse_edit_frame_move_y,
    mouse_edit_frame_move_z,
    mouse_edit_frame_rotate_x,
    mouse_edit_frame_rotate_y,
    mouse_edit_frame_rotate_z,
};

MouseAction mouse_action = mouse_none; ///< mouse current action
vec2i mouse_start; ///< mouse start/clicked location
vec2i mouse_last; ///< mouse last location (during dragging)

/// mouse motion event handler
void motion(int x, int y) {
    auto mouse = vec2i{x,y};
    auto delta = mouse - mouse_last;
    auto delta_f = vec2f{delta} * 0.01;
    switch(mouse_action) {
        case mouse_turntable_rotate: camera_view_turntable_rotate(scene->camera, -delta_f.x, -delta_f.y); break;
        case mouse_turntable_dolly: camera_view_turntable_dolly(scene->camera,-delta_f.y); break;
        case mouse_turntable_pan: camera_view_turntable_pan(scene->camera,-delta_f.x,delta_f.y); break;
        case mouse_edit_frame_move_x: selection_move(x3f*delta_f.y); break;
        case mouse_edit_frame_move_y: selection_move(y3f*delta_f.y); break;
        case mouse_edit_frame_move_z: selection_move(z3f*delta_f.y); break;
        case mouse_edit_frame_rotate_x: selection_rotate(x3f*delta_f.y); break;
        case mouse_edit_frame_rotate_y: selection_rotate(y3f*delta_f.y); break;
        case mouse_edit_frame_rotate_z: selection_rotate(z3f*delta_f.y); break;
        default: break;
    }
    mouse_last = vec2i(x,y);
    glutPostRedisplay();
}

/// mouse button event handler
void mouse(int button, int state, int x, int y) {
    mouse_start = vec2i(x,y);
    mouse_last = mouse_start;
    mouse_action = mouse_none;
    if(not state == GLUT_DOWN) return;
    
    if(glutGetModifiers() & GLUT_ACTIVE_ALT) {
        if(glutGetModifiers() & GLUT_ACTIVE_SHIFT) {
            if(button == GLUT_LEFT_BUTTON) mouse_action = mouse_edit_frame_rotate_x;
            if(button == GLUT_RIGHT_BUTTON) mouse_action = mouse_edit_frame_rotate_y;
            if(button == GLUT_MIDDLE_BUTTON) mouse_action = mouse_edit_frame_rotate_z;
            if(glutGetModifiers() & GLUT_ACTIVE_CTRL) {
                if(button == GLUT_LEFT_BUTTON) mouse_action = mouse_edit_frame_rotate_z;
                if(button == GLUT_RIGHT_BUTTON) mouse_action = mouse_edit_frame_rotate_z;
            }
        } else {
            if(button == GLUT_LEFT_BUTTON) mouse_action = mouse_edit_frame_move_x;
            if(button == GLUT_RIGHT_BUTTON) mouse_action = mouse_edit_frame_move_y;
            if(button == GLUT_MIDDLE_BUTTON) mouse_action = mouse_edit_frame_move_z;
            if(glutGetModifiers() & GLUT_ACTIVE_CTRL) {
                if(button == GLUT_LEFT_BUTTON) mouse_action = mouse_edit_frame_move_z;
                if(button == GLUT_RIGHT_BUTTON) mouse_action = mouse_edit_frame_move_z;
            }
        }
    } else {
        if(button == GLUT_LEFT_BUTTON) mouse_action = mouse_turntable_rotate;
        if(button == GLUT_RIGHT_BUTTON) mouse_action = mouse_turntable_dolly;
        if(button == GLUT_MIDDLE_BUTTON) mouse_action = mouse_turntable_pan;
        if(glutGetModifiers() & GLUT_ACTIVE_CTRL) {
            if(button == GLUT_LEFT_BUTTON) mouse_action = mouse_turntable_pan;
            if(button == GLUT_RIGHT_BUTTON) mouse_action = mouse_turntable_pan;
        }
    }
    
    motion(x,y);
}

/// draw a string using glut
void draw_string(const string& name) {
    for(auto c : name) glutBitmapCharacter(GLUT_BITMAP_8_BY_13, c);
}

/// draw hud
void display_hud() {
    char buf[2048];
    sprintf(buf, "time: %6d / draw: %s%s%s%s%s / light: %s / tess: %2d%s",
            (int)round(draw_opts.time*1000),
            (draw_opts.faces)?"f":" ",(draw_opts.edges)?"e":" ",
            (draw_opts.lines)?"l":" ",(draw_opts.control)?"c":" ",
            (draw_opts.doublesided)?"d":" ",
            (draw_opts.cameralights)?"c":"s",
            tesselation_level,(tesselation_smooth)?"s":"f");
//    msg01 += "/"+to_string("%3d",(int)round(1/hud_fps_display.elapsed()));
//    msg01 += "/"+to_string("%3d",(animating)?(int)round(1/hud_fps_update.elapsed()):0);
    
    int w = glutGet(GLUT_WINDOW_WIDTH);
    int h = glutGet(GLUT_WINDOW_HEIGHT);
    
    glColor3f(1,1,1);
    glPushMatrix();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0, w, h, 0);
    glMatrixMode(GL_MODELVIEW);
    glRasterPos2f(5, 20);
    draw_string(buf);
    glPopMatrix();
}

/// main draw method
void display() {
    hud_fps_display.start();
    if(draw_opts.cameralights) scene_cameralights_update(scene,draw_opts.cameralights_dir,draw_opts.cameralights_col);
    draw_scene(scene,draw_opts,true);
    draw_scene_decorations(scene,draw_opts,false);
    glFlush();
    hud_fps_display.stop();
    
    if(selected_frame) {
        auto axes = Axes();
        axes.frame = *selected_frame;
        draw_gizmo(&axes);
        if(selected_point) {
            auto dot = Dot();
            dot.pos = transform_point(*selected_frame, *selected_point);
            draw_gizmo(&dot);
        }
    }
    
    if(hud) display_hud();
    
    glutSwapBuffers();
    
    if(screenshotAndExit) {
        if(time_init_advance <= 0.0f or draw_opts.time >= time_init_advance ) {
            screenshot(filename_image.c_str());
            exit(0);
        }
    }
}

/// reshape event handler
void reshape(int w, int h) {
    draw_opts.res = h;
    camera_image_set_aspectratio(scene->camera, w, h);
}

/// idle event handler
void idle() {
    if(animating) {
        hud_fps_update.start();
        if(animating_steps) animate_step(true);
        else animate_update();
        hud_fps_update.stop();
        if(not animating) hud_fps_update.reset();
        glutPostRedisplay();
    }
#ifdef AUTORELOAD
    if(reload_auto) {
        static int tm = 0;
        struct stat st; 
        stat(filename_scene.c_str(),&st);
        if(tm != st.st_mtime) reload();
        tm = st.st_mtime;
    }
#endif
}

/// glut initialization
void init(int* argc, char** argv) {
	glutInit(argc, argv);
    char buf[2048];
    sprintf(buf,"rgba depth double samples<%d",draw_opts.samples);
    glutInitDisplayString(buf);
	glutInitWindowPosition(0,0);
    int w = camera_image_width(scene->camera,draw_opts.res);
    int h = camera_image_height(scene->camera,draw_opts.res);
    glutInitWindowSize(w,h);
    auto title = "view | " + filename_scene;
    glutCreateWindow(title.c_str());
	glutDisplayFunc(display);
	glutKeyboardFunc(keyboard);
	glutMouseFunc(mouse);
	glutMotionFunc(motion);
    glutReshapeFunc(reshape);
    glutIdleFunc(idle);
}

/// glut start main loop
void run() { glutMainLoop(); }

/// parses command line arguments
void parse_args(int argc, char** argv) {
	try {  
        TCLAP::CmdLine cmd("view", ' ', "0.0");
        
        TCLAP::ValueArg<int> resolutionArg("r","resolution","Image resolution",false,512,"resolution",cmd);
        TCLAP::ValueArg<int> samplesArg("s","samples","Image samples",false,4,"samples",cmd);
        
        TCLAP::SwitchArg hudArg("j","hud","HUD",cmd);
        TCLAP::SwitchArg screenshotAndExitArg("i","screenshotAndExit","Screenshot and exit",cmd);
        TCLAP::ValueArg<float> timeArg("t","time","Time advance (delays screenshot and exit)",false,0,"seconds",cmd);
        
        TCLAP::UnlabeledValueArg<string> filenameScene("scene","Scene filename",true,"","scene",cmd);
        TCLAP::UnlabeledValueArg<string> filenameImage("image","Image filename",false,"","image",cmd);
        
        cmd.parse( argc, argv );
        
        if(resolutionArg.isSet()) draw_opts.res = resolutionArg.getValue();
        if(samplesArg.isSet()) draw_opts.samples = samplesArg.getValue();
        if(hudArg.isSet()) hud = not hudArg.getValue();
        if(screenshotAndExitArg.isSet()) screenshotAndExit = screenshotAndExitArg.getValue();
        if(timeArg.isSet()) time_init_advance = timeArg.getValue();
        
        filename_scene = filenameScene.getValue();
        if(filenameImage.isSet()) filename_image = filenameImage.getValue();
        else { filename_image = filename_scene.substr(0,filename_scene.length()-4)+"png"; }
	} catch (TCLAP::ArgException &e) { 
        std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl; 
    }
}

/// main: parses args, loads scene, start gui
int main(int argc, char** argv) {
    parse_args(argc, argv);
    load();
	init(&argc, argv);

    if( time_init_advance > 0.0f ) {
        animate_start();
        //draw_opts.time = time_init_advance;
        //scene_simulation_update(scene,time_init_advance);
        //if(simulate_has) scene_simulation_update(scene,time_init_advance);
    }

    run();
}

///@}

