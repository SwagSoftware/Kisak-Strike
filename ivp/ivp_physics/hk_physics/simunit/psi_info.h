class hk_PSI_Info: protected IVP_Event_Sim {
public:
	hk_PSI_Info(const IVP_Event_Sim &sim) : IVP_Event_Sim(sim) {}

	hk_real get_inv_delta_time(){
		return i_delta_time;
	}

	hk_real get_delta_time(){
		return delta_time;
	}

};
