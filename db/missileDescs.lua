AAMDescs = {
	DefaultAAM = {
		motorBurnTime		= 6,	-- motor op time
		motorSpecImpulse	= 235,	-- motor Isp
		motorFuelMass		= 60,	-- motor fuel mass
		maxNormAccel		= 30,	-- maximum normal acceleration
		emptyMass			= 230,	-- missile's empty mass
		planformArea		= 0.9,	-- missile's characteristic/planform area
		DyPerDa				= 1.5,	-- amount of Fy generated per ° of AoA
		proxyFuzeRadius		= 15,	-- proxy fuze's trigger radius
		seekerMaxOBA		= 15,	-- seeker's one-side FoV
		navConstant			= 1.5,	-- AP's guidance/navigation constant
		-- .5M, .9M, 1.2M, 1.5M, 2M, 3M, 4M (7 values in total)
		cXData = { 0.012, 0.015, 0.046, 0.044, 0.038, 0.030, 0.026 };
	},
}
