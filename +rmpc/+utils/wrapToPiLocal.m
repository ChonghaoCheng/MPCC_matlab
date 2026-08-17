function a = wrapToPiLocal(a)
a = mod(a + pi, 2 * pi) - pi;
end
