function time = GetTimeStamp( message )
%HEADERTIMESTAMP Summary of this function goes here
%   Detailed explanation goes here
  t = message.Header.Stamp;
  time = double(t.Sec)+double(t.Nsec)*10^-9;
end

