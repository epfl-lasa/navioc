% Create scripts for running a test.
function testwrite(name,call,A,S,R,stitch_only,create_sh,precompile,corn,long_queue)

% General boilerplate.
% Torque job boilerplate.
tjboiler = 'this should be filled in: -N %s matlab -r %s';
stitchboiler = 'series_result = [];transfer_result = [];\n';
stitchend = 'saveresults(''%s'',test_params,test_metric_names,mdp_params,mdp_cat_name,mdp_param_names,algorithms,names,colors,order,restarts,series_result,transfer_result);';

% Generate code for each of the tests.
if ~stitch_only,
    % Create running code.
    if create_sh,
        runfid = fopen(['autorun_' name '_qsub.sh'],'w');
    end;
end;
% Create boilerplate for stitching code.
stitchfid = fopen(['auto_' name '_stitch.m'],'w');
fprintf(stitchfid,stitchboiler);
for a=1:A,
    if create_sh,
        runafid = fopen(['autorun_' name '_' num2str(a) '_qsub.sh'],'w');
        fprintf(runfid,['./autorun_' name '_' num2str(a) '_qsub.sh\n']);
    end;
    for s=1:S,
        for r=1:R,
            % Name for this line.
            rname = [name '_' num2str(a) '_' num2str(s) '_' num2str(r)];
            % Write stitching line.
            if a == 1 && s == 1 && r == 1,
                fprintf(stitchfid,'load %s\n',rname);
            end;
            fprintf(stitchfid,'series_result = appendseries(%i,%i,%i,series_result,''%s'');\n',a,s,r,...
                [rname '.mat']);
            if ~stitch_only,
                if create_sh,
                    % Write torque script.
                    torquefid = fopen(['autorun_' rname '.sh'],'w');
                    if precompile,
                        fullcall = sprintf(call,num2str(a),num2str(s),num2str(r),name);
                        fprintf(torquefid,tjboiler,rname,fullcall);
                    else
                        fprintf(torquefid,tjboiler,rname,['auto_' rname]);
                    end;
                    fclose(torquefid);
                    if long_queue && strcmp(corn,'corn'),
                        fprintf(runafid,'qsub -q long.q %s\n',['autorun_' rname '.sh']);
                    else
                        fprintf(runafid,'qsub %s\n',['autorun_' rname '.sh']);
                    end;
                end;
                if ~precompile,
                    % Write .m script.
                    mfid = fopen(['auto_' rname '.m'],'w');
                    if ~precompile,
                        fprintf(mfid,'addpaths;\n');
                    end;
                    fprintf(mfid,[call ';\n'],num2str(a),num2str(s),num2str(r),name);
                    % Close files.
                    fclose(mfid);
                end;
            end;
        end;
    end;
    if create_sh,
        fclose(runafid);
    end;
end;
% Finish stitching code.
fprintf(stitchfid,stitchend,name);
fclose(stitchfid);
if ~stitch_only,
    if create_sh,
        fclose(runfid);
    end;
end;

if ~stitch_only && ~precompile,
    % Create boilerplate code for running everything.
    allfid = fopen(['auto_' name '_all.m'],'w');
    fprintf(allfid,'addpaths;\n');
    fprintf(allfid,'matlabpool open %i;\n',R);
    fprintf(allfid,'parfor r=1:%i,\n',R);
    fprintf(allfid,'    for a=1:%i,\n',A);
    fprintf(allfid,'        for s=1:%i,\n',S);
    fprintf(allfid,['            ' call ';\n'],'a','s','r',name);
    fprintf(allfid,'        end;\n');
    fprintf(allfid,'    end;\n');
    fprintf(allfid,'end;\n');
    fprintf(allfid,'matlabpool close;\n');
    fprintf(allfid,'auto_%s_stitch;\n',name);
    fclose(allfid);
end;
