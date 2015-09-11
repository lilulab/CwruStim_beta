function [kneestimtics] = FNSkneetics(nchan,GCl,GCr,SPl,SPr,MA)
% by Curtis S. To
% date created: 041109
% date updated: 041109
% [kneestimtics] = FNSkneetics(nchan,GCl,GCr,SPl,SPr,MA)
% determines the time instances (in percentage gait cycle) in which the 
% knee flexors and extensors are stimulated.  This information will be used 
% to determine when each knee constraint can be unlocked and locked.  The
% vector, kneestimtics, contains the time instances (in percentage gait
% cycle) for stimulation onset as follows [right flex stim, left flex stim,
% right extend stim, left extend stim].  The tic values are modulated in
% the HNP Exoskeleton FNS Feedback Signal Simulink block to account for the
% knee mechanism response time.
%--------------------------------------------------------------------------

% Resize gait cycle and stimulation parameter vectors into matrices with
% columns as time and rows as channel numbers:
ntic = length(GCl) / nchan;    % number of tics
for n = 1:nchan
    GClm(n,:) = GCl((n-1)*ntic+1:n*ntic)';  % left gait cycle
    SPlm(n,:) = SPl((n-1)*ntic+1:n*ntic)';  % left stimulation parameter
    GCrm(n,:) = GCr((n-1)*ntic+1:n*ntic)';  % right gait cycle
    SPrm(n,:) = SPr((n-1)*ntic+1:n*ntic)';  % right stimulation parameter
end

kneestimtics = [100 100 100 100];   % initialize knee constraint tics
                                    % [right flex stim, left flex stim, ...
                                    % [right extend stim, left extend stim]
for ch = 1:nchan
    if MA(ch,2) == 2	% right knee flexion
        for n = 2:length(SPrm(ch,:))
        	if (SPrm(ch,n-1) == 0) && (SPrm(ch,n) > 0) ...  % find when start flex stim
                && (GCrm(ch,n-1) < kneestimtics(1)) % find earliest flexion activation
                kneestimtics(1) = GCrm(ch,n-1);     % assign unlock tic
            end
        end         
    end
    if MA(ch,2) == 5 	% left knee flexion
        for n = 2:length(SPlm(ch,:))
        	if (SPlm(ch,n-1) == 0) && (SPlm(ch,n) > 0) ...  % find when start flex stim
                && (GClm(ch,n-1) < kneestimtics(2)) % find earliest flexion activation
                kneestimtics(2) = GClm(ch,n-1);     % assign unlock tic
            end
        end         
    end
    if MA(ch,2) == -2	% right knee extension
        for n = 2:length(SPrm(ch,:))
        	if (SPrm(ch,n-1) == 0) && (SPrm(ch,n) > 0) ...  % find when start extend stim
                && (GCrm(ch,n-1) < kneestimtics(3))	% find earliest extension activation
                kneestimtics(3) = GCrm(ch,n-1);     % assign unlock tic
            end
        end         
    end
    if MA(ch,2) == -5 	% left knee extension
        for n = 2:length(SPlm(ch,:))
        	if (SPlm(ch,n-1) == 0) && (SPlm(ch,n) > 0) ...  % find when start extend stim
                && (GClm(ch,n-1) < kneestimtics(4)) % find earliest extension activation
                kneestimtics(4) = GClm(ch,n-1);     % assign unlock tic
            end
        end         
    end
end