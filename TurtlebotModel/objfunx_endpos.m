function f = objfunx_endpos(x_chaser_kpN,Q, x_target_kpN)
f = (x_target_kpN - x_chaser_kpN)' * Q * (x_target_kpN - x_chaser_kpN);
end