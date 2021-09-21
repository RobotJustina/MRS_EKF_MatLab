rosshutdown
rosinit

tftree = rostf
pause(2);

idxs = find(contains(tftree.AvailableFrames,'ar_'));

for j = 1:10

    for i = 1:length(idxs)
	try
		mark_i = string( tftree.AvailableFrames(idxs(i)) )

		markToMap = getTransform(tftree, mark_i, 'map');
		markToMapTranslation = markToMap.Transform.Translation;

		markToBase = getTransform(tftree, mark_i, 'base_link');
		markToBaseTranslation = markToBase.Transform.Translation;

		baseToMap = getTransform(tftree, 'base_link', 'map');
		baseToMapTranslation = baseToMap.Transform.Translation;
	catch
		warning('No TF')
	end
    end
    pause(2);
end
