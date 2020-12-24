	project "Bullet3Common"

	language "C++"
				
	kind "StaticLib"
		
	if os.istarget("Linux") then
	    buildoptions{"-fPIC"}
	end

	includedirs {".."}

	files {
		"*.cpp",
		"*.h"
	}
