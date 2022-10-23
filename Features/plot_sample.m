function plot_sample(sample, mdp_data)
    v_des = mdp_data.v_des;
    hold on
    for i = 1:mdp_data.n_ped
        X = sample.states(:, 2*i-1);
        Y = sample.states(:, 2*i);
        h = plot(X, Y, 'Marker', 'o');
        c = get(h, 'Color');
        quiver(X(end), Y(end), v_des(2*i-1), v_des(2*i), 'Color', c);
    end
    xlim(2*[-mdp_data.half_width, mdp_data.half_width])
    ylim(4*[-mdp_data.half_height, mdp_data.half_height])
    daspect([1,1,1])
end
