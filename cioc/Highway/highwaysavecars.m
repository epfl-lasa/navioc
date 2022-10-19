% Save cars info to a file.
function highwaysavecars(mdp_data,file)

loaded_cars = mdp_data.cars;
save(file,'loaded_cars');
