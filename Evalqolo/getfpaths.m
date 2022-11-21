function fpaths = getfpaths()

fpaths = struct(...
	'crowd', struct(...
		'navioc', [
			"crowd/matfiles/2022-11-18-10-04-05.mat"
			"crowd/matfiles/2022-11-18-10-06-27.mat"
			"crowd/matfiles/2022-11-18-10-09-07.mat"
			"crowd/matfiles/2022-11-18-10-11-21.mat"
			"crowd/matfiles/2022-11-18-10-14-10.mat"
			"crowd/matfiles/2022-11-18-10-16-54.mat"
			"crowdNoon/naviocNoon/matfiles/2022-11-18-13-23-54.mat"], ...
		'comb', [
			"crowdNoon/combNoon/matfiles/2022-11-18-12-50-48.mat"
			"crowdNoon/combNoon/matfiles/2022-11-18-12-53-04.mat"
			"crowdNoon/combNoon/matfiles/2022-11-18-12-55-31.mat"
			"crowdNoon/combNoon/matfiles/2022-11-18-12-59-20.mat"
			"crowdNoon/combNoon/matfiles/2022-11-18-13-01-37.mat"
			"crowdNoon/combNoon/matfiles/2022-11-18-13-03-56.mat"
			"crowdNoon/combNoon/matfiles/2022-11-18-13-06-17.mat"
			"crowdNoon/combNoon/matfiles/2022-11-18-13-08-21.mat"
			"crowdNoon/combNoon/matfiles/2022-11-18-13-10-50.mat"
			"crowdNoon/combNoon/matfiles/2022-11-18-13-13-00.mat"
			"crowdNoon/combNoon/matfiles/2022-11-18-13-16-02.mat"], ...
		'orca', [
			"crowdNoon/ORCAfails/matfiles/2022-11-18-12-31-25.mat"
			"crowdNoon/ORCAfails/matfiles/2022-11-18-12-34-49.mat"
			"crowdNoon/ORCAfails/matfiles/2022-11-18-12-39-09.mat"
			"crowdNoon/ORCAfails/matfiles/2022-11-18-12-41-30.mat"
			"crowdNoon/ORCAfails/matfiles/2022-11-18-12-43-31.mat"]), ...
	'static', struct(...
		'navioc', [
			"static_obstacle/matfiles/2022-11-17-23-24-56.mat"
			"static_obstacle/matfiles/2022-11-18-09-52-59.mat"], ...
		'comb', [
			"static_obstacle/matfiles/2022-11-18-09-44-15.mat"
			"static_obstacle/matfiles/2022-11-18-09-47-05.mat"], ...
		'orca', [
			"static_obstacle/matfiles/2022-11-17-23-13-03.mat"
			"static_obstacle/matfiles/2022-11-18-09-57-06.mat"]) ...
);