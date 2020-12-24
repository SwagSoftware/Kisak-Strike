	project "Bullet2FileLoader"
		
	kind "StaticLib"
	
	includedirs {
		"../../../src"
	}
	
    if os.istarget("Linux") then
        buildoptions{"-fPIC"}
    end
	 
	files {
		"**.cpp",
		"**.h"
	}