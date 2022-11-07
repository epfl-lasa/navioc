function names = fnames(res)

names = [];
for i = 1:length(res.irl_result.reward.features)
    names = [names, res.irl_result.reward.features{i}.type, ' '];
end

end

