//IVP_EXPORT_PUBLIC

#define IVP_WHITESPACE " \t,;\n"

class P_String {
public:
    static const char *find_string(const char *str, const char *key, int upper_case_flag);
    static void uppercase(char *string_to_change);	// in place
    static int string_cmp(const char *str,const char *search,IVP_BOOL upper_case);
	/*	*	Wildcard in search string */
	/*	?	any	Charakter	*/
	/*	if uppercase	change all letters to uppercase */
	/*	returns 0 if strings are equal -1 or +1 if left strings is
		less/greater than right string */


};


extern IVP_ERROR_STRING 	p_export_error(const char *templat, ...);
extern IVP_ERROR_STRING 	p_get_error();
extern void	p_print_error();
extern void 	ivp_message(const char *templat, ...);
extern void 	p_error_message();
extern void 	p_error_message(const char *templat, ...);
extern char *	p_make_string(const char *templat, ...);
extern char *	p_make_string_fast(const char *templat, ...); // doesn't check overflow

int p_strcmp(const char *s1, const char *s2);
int p_strlen(const char *s);
char *p_read_first_token(FILE *fp);
char *p_get_next_token();
int   p_get_num();
IVP_DOUBLE	 p_get_float();
char *p_get_string();
char *p_str_tok(char *str,const char *delimiter);

int p_atoi(const char *value);		// converts ascii to int
IVP_DOUBLE p_atof(const char *s);

#ifdef WIN32
int strcasecmp(const char *a, const char *b);
void replace_slash(const char *in,char *out);
long p_get_time(); // returns seconds since 1970
#endif






