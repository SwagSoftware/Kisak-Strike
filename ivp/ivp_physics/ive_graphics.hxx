// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PUBLIC

// used for graphical output only

class P_Hardware;
class IVP_Compact_Surface;
class P_Texture_Server;
class IVP_Example_Base;


// GRAPHICS SUPPORT

extern char debugstring[1024]; //for text output

// register your own demo texture server
extern void p_demo_register_texture_server(P_Hardware *hw, P_Texture_Server *server);

// initialize display engine (before creating objects),
// the display engine will listen to the environment for new objects
extern void p_init_demo_display(P_Hardware *hw, IVP_Environment *environment);

// manually set the matrix for the camera
extern void p_set_demo_camera_matrix(IVP_U_Matrix *mat);

// used to visualize phantom objects
extern void p_demo_put_object_in_moving_list(IVP_Real_Object *real_object);

// update graphics for a frozen object (useful after e.g. beaming the object)
extern void p_demo_update_graphics_for_frozen_object(IVP_Real_Object *real_object);

// render graphics
extern void p_demo_render_scene(P_Hardware *hw, IVP_Environment *env = NULL);

// never render the object that is to be created next
extern void p_demo_dont_render_next_object();

//
#if !defined(_XBOX) && (!defined(__MWERKS__) || !defined(__POWERPC__)) && !defined(GEKKO)
extern void p_demo_replace_graphical_surface(const IVP_Compact_Surface *graphical_surface);
#else // Xbox or Mac or GameCube
extern void p_demo_replace_graphical_surface(IVP_Real_Object* obj, const IVP_Compact_Surface *graphical_surface);
#endif
//
extern void p_graphlib_render_scene_splitscreen(P_Hardware *hw);

//
extern void p_graphlib_init_splitscreen(P_Hardware *hw, IVP_Environment *environment);

//
extern void p_graphlib_set_camera_matrix_split_screen(IVP_U_Matrix *mat, int frame_nr);

//
extern void p_demo_create_ambientlight(IVP_U_Point light_color);
extern void p_demo_create_pointlight(IVP_U_Point light_color, IVP_U_Point position);
extern void p_demo_move_pointlight(IVP_U_Point position);

//
extern void p_demo_enable_mousepointer(P_Hardware *hardware);
extern void p_demo_disable_mousepointer(P_Hardware *hardware);

//
class IVP_Ray_Solver_Min_Hash;
class IVP_U_Min_Hash;
extern int p_mousepointer_raycast(P_Hardware *hardware, IVP_Environment *environment, IVP_U_Matrix *camera, IVP_U_Point *ray_direction, IVP_Ray_Solver_Min_Hash **raysolver_minhash, IVP_U_Min_Hash **min_hash);

// *** SOUND SUPPORT
extern void p_init_sound(P_Hardware *hw, char *levelname);
extern void p_exit_sound(P_Hardware *hw);
extern void p_demo_make_motor_sound_loud();
extern void p_demo_refresh_sound();
extern void p_demo_refresh_vehicle_sound(IVP_FLOAT rot_speed);



// *** STATISTICAL OUTPUT
class IVP_BetterStatisticsmanager_Callback_Interface;
IVP_BetterStatisticsmanager_Callback_Interface *p_demo_get_statisticsmanager_callback(P_Hardware *hardware);


    
extern IVP_BOOL p_demo_get_shift_key_state(P_Hardware *hw);

// return the key pressed
extern int p_demo_eval_key_input(P_Hardware *hw);

// return the time needed to draw a frame (clipped [0 .. 0.1f] sec)
extern IVP_DOUBLE p_demo_get_delta_time(IVP_Environment *environment);

// get a active IVP_FLOAT manager which already manages some keyboard driven active floats
extern IVP_U_Active_Value_Manager *p_demo_get_active_float_manager();

// import DirectX object into Ipion engine
class IVP_Concave_Polyhedron;
extern int p_graphlib_convert_directx_object_to_concave_data(P_Hardware *hw, const char *filename, IVP_Concave_Polyhedron *concave_polyhedron);
extern int p_graphlib_robust_convert_directx_object_to_compact_ledges(P_Hardware *hw, const char *filename, IVP_U_BigVector<IVP_Compact_Ledge> *ledges);


enum IVP_EXAMPLE_MENU_CLASS {
    IVP_EMC_COMPLEX,
    IVP_EMC_BASIC,
    IVP_EMC_QUICKSTART,
    IVP_EMC_DEVELOPMENT,
    IVP_EMC_BUG,
    IVP_EMC_OLD,
    IVP_EMC_NONE
};

// basic class used as an interface between the main() function and the example codes
class IVP_Example_Base {
public:
    // garbage collection
    IVP_U_Vector<IVP_Compact_Surface>   gc_compactsurface;
    IVP_U_Vector<IVP_SurfaceManager>    gc_surfacemanager;
    IVP_U_Vector<IVP_Material>		gc_material;

    // garbage collection
    void gc_cleanup();

    const char *my_name;    // the name (on the command line of the example)
    const char *my_level_name;  // the predifened world to load (see directory IVP_Sample_Data/LEVELS/
    const char *my_helptext; // optional help text when hitting SPACE (NULL: no helptext);
    const char *my_introtext; // optional intro text before simulation (NULL: no introtext)
    IVP_EXAMPLE_MENU_CLASS my_class;

    IVP_BOOL import_external_environment; // should the demo_level environment be physicalized
    IVP_Environment *environment;		// set only if import_external_environment==TRUE
    P_Hardware *hardware;

    // register_to_main should be called by the constructor to register
    // the example to the ::main() function
    void register_to_main(IVP_BOOL import_external_environment = IVP_FALSE);

    IVP_Example_Base(IVP_EXAMPLE_MENU_CLASS example_class);

    virtual ~IVP_Example_Base();

	// main is called by any program that wants to give up controll untill end of example
	// but if you want to have more interactive controll of the example, eg us GLUT which
	// has its own render loop, call setup(), step() multiple times ,and then cleanup()  
    // CALL ONE SET OF FUNCTIONS, BUT NOT BOTH..

    virtual int main();    //old technique, used in hull_editor
    
    virtual void setup()=0; // the examples setup function
    virtual void cleanup()=0; // the examples cleanup mem / end function
    virtual int  step()=0; // the examples step function; return -2 for restart!
};


class P_Texture_Server {
public:
    IVP_BOOL return_absolute_positions; // set this flag to IVP_TRUE if your texture callback will return absolute texture positions

    // called just before the first call to texture_callback
    virtual void texture_server_init(const IVP_Real_Object * /*real_object*/) = 0;

    // called after the last call of texture_callback
    virtual void texture_server_exit(const IVP_Real_Object * /*real_object*/) = 0;

    virtual void texture_callback_detail(const IVP_Real_Object * /*real_object_in*/,
				  const IVP_U_Point * /*p1_os_in*/,
				  const IVP_U_Point * /*p2_os_in*/,
				  const IVP_U_Point * /*p3_os_in*/,
				  const IVP_U_Point * /*normal_os_in*/,
				  const char ** /*texture_name_out*/,
				  IVP_FLOAT * /*u1_out*/,
				  IVP_FLOAT * /*v1_out*/,
				  IVP_FLOAT * /*u2_out*/,
				  IVP_FLOAT * /*v2_out*/,
				  IVP_FLOAT * /*u3_out*/,
				  IVP_FLOAT * /*v3_out*/
				  ) {;};

    virtual void texture_callback(const IVP_Real_Object * /*real_object_in*/,
				  const char ** /*texture_name_out*/,
				  IVP_FLOAT * /*scale_factor_out*/
				  ) {;};

    P_Texture_Server() {
	this->return_absolute_positions = IVP_FALSE;
    }
};


enum IVP_EXAMPLE_MOVING_CAMERA_MODE {
    IVP_EMCM_ROTATION,
    IVP_EMCM_SPACEFLIGHT
};


class Example_Moving_Camera {	// 
private:
    void cameramatrix_workaround();
    void convert_position_and_focus_to_angles_and_distance(const IVP_U_Point &center);

    // -----------------------------------------------------------------------------------
    // copy the following class to your class (private!) if you want to set up a callback.
    // then call camera.set_callback_relative(&relative_default); after instantiating
    // camera.
    // -----------------------------------------------------------------------------------
public:
    class Camera_Offset_Modifier {
    public:
	virtual IVP_U_Point *calc_offset_ws(IVP_Time current_time) = 0;
	// -----------------------------------------
	// get the relative matrix for offset_center
	// overload it for enhanced functionality
	// -----------------------------------------
    };
protected:
    Camera_Offset_Modifier *offset_modifier;
    int      current_mode;

public:

    void init();

    // ------------------------------------
    // you may set these variables by hand.
    // ------------------------------------

    IVP_BOOL manual_position_set;
    IVP_FLOAT distance;
    IVP_FLOAT angle_x;
    IVP_FLOAT angle_y;
    IVP_U_Point focus;
    IVP_U_Float_Point offset_center;

    IVP_FLOAT mouse_x_speed, mouse_y_speed;
    IVP_DOUBLE initial_speed, acceleration;
    IVP_U_Matrix m_world_f_camera;
    IVP_BOOL camera_is_pointlight;

    // ------------------------------------------
    // all these functions set the camera matrix:
    // ------------------------------------------

    void set_mode(IVP_EXAMPLE_MOVING_CAMERA_MODE mode);
    void update(IVP_DOUBLE d_time);

    // position camera after you have set m_world_f_camera manually
    void show_picture();
    
    void set_offset_modifier(Camera_Offset_Modifier *);

    // position camera, given camera position and viewing angles
    void show_picture(const IVP_U_Point &cameraposition, const IVP_U_Point &direction);
    void show_picture(const IVP_U_Point &cameraposition, const IVP_U_Point &direction, const IVP_U_Point &up);

    // position camera, given camera position and a visible point and optionally a vector where "up" is
    void show_worldpoint(const IVP_U_Point &cameraposition, const IVP_U_Point &worldpoint, const IVP_U_Point *up_direction = NULL);

    // a camera rotating around a given point
    void rotation(const IVP_U_Point &center, IVP_DOUBLE d_time, IVP_BOOL recalc_values = IVP_TRUE);

    // a camera moving through the level
    void space_movement(IVP_DOUBLE d_time);

    // a camera flying through the level (elite/frontier-like)
    void space_flight(IVP_DOUBLE d_time, IVP_DOUBLE flight_acceleration);
    void space_flight_original(IVP_DOUBLE d_time, IVP_BOOL recalc_values = IVP_TRUE);
    void space_flight_old(IVP_DOUBLE d_time);


    IVP_U_Point get_position();
    IVP_U_Point get_direction();
    IVP_U_Point get_upvec();

    IVP_Environment *env;
    Example_Moving_Camera(IVP_Environment &env);
    Example_Moving_Camera(IVP_Environment &env, const IVP_U_Point &position); // init where the camera is.
};



