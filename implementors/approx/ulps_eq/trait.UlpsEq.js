(function() {var implementors = {
"approx":[],
"palette":[["impl&lt;T: <a class=\"trait\" href=\"palette/float/trait.Float.html\" title=\"trait palette::float::Float\">Float</a> + <a class=\"trait\" href=\"palette/trait.FromF64.html\" title=\"trait palette::FromF64\">FromF64</a> + <a class=\"trait\" href=\"approx/ulps_eq/trait.UlpsEq.html\" title=\"trait approx::ulps_eq::UlpsEq\">UlpsEq</a>&gt; <a class=\"trait\" href=\"approx/ulps_eq/trait.UlpsEq.html\" title=\"trait approx::ulps_eq::UlpsEq\">UlpsEq</a>&lt;<a class=\"struct\" href=\"palette/struct.RgbHue.html\" title=\"struct palette::RgbHue\">RgbHue</a>&lt;T&gt;&gt; for <a class=\"struct\" href=\"palette/struct.RgbHue.html\" title=\"struct palette::RgbHue\">RgbHue</a>&lt;T&gt;<span class=\"where fmt-newline\">where\n    T::<a class=\"associatedtype\" href=\"approx/abs_diff_eq/trait.AbsDiffEq.html#associatedtype.Epsilon\" title=\"type approx::abs_diff_eq::AbsDiffEq::Epsilon\">Epsilon</a>: <a class=\"trait\" href=\"palette/float/trait.Float.html\" title=\"trait palette::float::Float\">Float</a> + <a class=\"trait\" href=\"palette/trait.FromF64.html\" title=\"trait palette::FromF64\">FromF64</a>,</span>"],["impl&lt;T&gt; <a class=\"trait\" href=\"approx/ulps_eq/trait.UlpsEq.html\" title=\"trait approx::ulps_eq::UlpsEq\">UlpsEq</a>&lt;<a class=\"struct\" href=\"palette/struct.Oklch.html\" title=\"struct palette::Oklch\">Oklch</a>&lt;T&gt;&gt; for <a class=\"struct\" href=\"palette/struct.Oklch.html\" title=\"struct palette::Oklch\">Oklch</a>&lt;T&gt;<span class=\"where fmt-newline\">where\n    T: <a class=\"trait\" href=\"palette/trait.FloatComponent.html\" title=\"trait palette::FloatComponent\">FloatComponent</a> + <a class=\"trait\" href=\"approx/ulps_eq/trait.UlpsEq.html\" title=\"trait approx::ulps_eq::UlpsEq\">UlpsEq</a>,\n    T::<a class=\"associatedtype\" href=\"approx/abs_diff_eq/trait.AbsDiffEq.html#associatedtype.Epsilon\" title=\"type approx::abs_diff_eq::AbsDiffEq::Epsilon\">Epsilon</a>: <a class=\"trait\" href=\"https://doc.rust-lang.org/1.71.0/core/marker/trait.Copy.html\" title=\"trait core::marker::Copy\">Copy</a> + <a class=\"trait\" href=\"palette/trait.FloatComponent.html\" title=\"trait palette::FloatComponent\">FloatComponent</a>,</span>"],["impl&lt;Wp, T&gt; <a class=\"trait\" href=\"approx/ulps_eq/trait.UlpsEq.html\" title=\"trait approx::ulps_eq::UlpsEq\">UlpsEq</a>&lt;<a class=\"struct\" href=\"palette/struct.Xyz.html\" title=\"struct palette::Xyz\">Xyz</a>&lt;Wp, T&gt;&gt; for <a class=\"struct\" href=\"palette/struct.Xyz.html\" title=\"struct palette::Xyz\">Xyz</a>&lt;Wp, T&gt;<span class=\"where fmt-newline\">where\n    T: <a class=\"trait\" href=\"palette/trait.FloatComponent.html\" title=\"trait palette::FloatComponent\">FloatComponent</a> + <a class=\"trait\" href=\"approx/ulps_eq/trait.UlpsEq.html\" title=\"trait approx::ulps_eq::UlpsEq\">UlpsEq</a>,\n    T::<a class=\"associatedtype\" href=\"approx/abs_diff_eq/trait.AbsDiffEq.html#associatedtype.Epsilon\" title=\"type approx::abs_diff_eq::AbsDiffEq::Epsilon\">Epsilon</a>: <a class=\"trait\" href=\"https://doc.rust-lang.org/1.71.0/core/marker/trait.Copy.html\" title=\"trait core::marker::Copy\">Copy</a> + <a class=\"trait\" href=\"palette/trait.FloatComponent.html\" title=\"trait palette::FloatComponent\">FloatComponent</a>,\n    Wp: <a class=\"trait\" href=\"palette/white_point/trait.WhitePoint.html\" title=\"trait palette::white_point::WhitePoint\">WhitePoint</a> + <a class=\"trait\" href=\"https://doc.rust-lang.org/1.71.0/core/cmp/trait.PartialEq.html\" title=\"trait core::cmp::PartialEq\">PartialEq</a>,</span>"],["impl&lt;Wp, T&gt; <a class=\"trait\" href=\"approx/ulps_eq/trait.UlpsEq.html\" title=\"trait approx::ulps_eq::UlpsEq\">UlpsEq</a>&lt;<a class=\"struct\" href=\"palette/struct.Hsluv.html\" title=\"struct palette::Hsluv\">Hsluv</a>&lt;Wp, T&gt;&gt; for <a class=\"struct\" href=\"palette/struct.Hsluv.html\" title=\"struct palette::Hsluv\">Hsluv</a>&lt;Wp, T&gt;<span class=\"where fmt-newline\">where\n    T: <a class=\"trait\" href=\"palette/trait.FloatComponent.html\" title=\"trait palette::FloatComponent\">FloatComponent</a> + <a class=\"trait\" href=\"approx/ulps_eq/trait.UlpsEq.html\" title=\"trait approx::ulps_eq::UlpsEq\">UlpsEq</a>,\n    T::<a class=\"associatedtype\" href=\"approx/abs_diff_eq/trait.AbsDiffEq.html#associatedtype.Epsilon\" title=\"type approx::abs_diff_eq::AbsDiffEq::Epsilon\">Epsilon</a>: <a class=\"trait\" href=\"https://doc.rust-lang.org/1.71.0/core/marker/trait.Copy.html\" title=\"trait core::marker::Copy\">Copy</a> + <a class=\"trait\" href=\"palette/trait.FloatComponent.html\" title=\"trait palette::FloatComponent\">FloatComponent</a>,\n    Wp: <a class=\"trait\" href=\"palette/white_point/trait.WhitePoint.html\" title=\"trait palette::white_point::WhitePoint\">WhitePoint</a> + <a class=\"trait\" href=\"https://doc.rust-lang.org/1.71.0/core/cmp/trait.PartialEq.html\" title=\"trait core::cmp::PartialEq\">PartialEq</a>,</span>"],["impl&lt;T&gt; <a class=\"trait\" href=\"approx/ulps_eq/trait.UlpsEq.html\" title=\"trait approx::ulps_eq::UlpsEq\">UlpsEq</a>&lt;<a class=\"struct\" href=\"palette/gradient/struct.Range.html\" title=\"struct palette::gradient::Range\">Range</a>&lt;T&gt;&gt; for <a class=\"struct\" href=\"palette/gradient/struct.Range.html\" title=\"struct palette::gradient::Range\">Range</a>&lt;T&gt;<span class=\"where fmt-newline\">where\n    T: <a class=\"trait\" href=\"approx/ulps_eq/trait.UlpsEq.html\" title=\"trait approx::ulps_eq::UlpsEq\">UlpsEq</a> + <a class=\"trait\" href=\"palette/float/trait.Float.html\" title=\"trait palette::float::Float\">Float</a>,\n    T::<a class=\"associatedtype\" href=\"approx/abs_diff_eq/trait.AbsDiffEq.html#associatedtype.Epsilon\" title=\"type approx::abs_diff_eq::AbsDiffEq::Epsilon\">Epsilon</a>: <a class=\"trait\" href=\"https://doc.rust-lang.org/1.71.0/core/marker/trait.Copy.html\" title=\"trait core::marker::Copy\">Copy</a>,</span>"],["impl&lt;S, T&gt; <a class=\"trait\" href=\"approx/ulps_eq/trait.UlpsEq.html\" title=\"trait approx::ulps_eq::UlpsEq\">UlpsEq</a>&lt;<a class=\"struct\" href=\"palette/struct.Hsl.html\" title=\"struct palette::Hsl\">Hsl</a>&lt;S, T&gt;&gt; for <a class=\"struct\" href=\"palette/struct.Hsl.html\" title=\"struct palette::Hsl\">Hsl</a>&lt;S, T&gt;<span class=\"where fmt-newline\">where\n    T: <a class=\"trait\" href=\"palette/trait.FloatComponent.html\" title=\"trait palette::FloatComponent\">FloatComponent</a> + <a class=\"trait\" href=\"approx/ulps_eq/trait.UlpsEq.html\" title=\"trait approx::ulps_eq::UlpsEq\">UlpsEq</a>,\n    T::<a class=\"associatedtype\" href=\"approx/abs_diff_eq/trait.AbsDiffEq.html#associatedtype.Epsilon\" title=\"type approx::abs_diff_eq::AbsDiffEq::Epsilon\">Epsilon</a>: <a class=\"trait\" href=\"https://doc.rust-lang.org/1.71.0/core/marker/trait.Copy.html\" title=\"trait core::marker::Copy\">Copy</a> + <a class=\"trait\" href=\"palette/float/trait.Float.html\" title=\"trait palette::float::Float\">Float</a> + <a class=\"trait\" href=\"palette/trait.FromF64.html\" title=\"trait palette::FromF64\">FromF64</a>,\n    S: <a class=\"trait\" href=\"palette/rgb/trait.RgbStandard.html\" title=\"trait palette::rgb::RgbStandard\">RgbStandard</a> + <a class=\"trait\" href=\"https://doc.rust-lang.org/1.71.0/core/cmp/trait.PartialEq.html\" title=\"trait core::cmp::PartialEq\">PartialEq</a>,</span>"],["impl&lt;Wp, T&gt; <a class=\"trait\" href=\"approx/ulps_eq/trait.UlpsEq.html\" title=\"trait approx::ulps_eq::UlpsEq\">UlpsEq</a>&lt;<a class=\"struct\" href=\"palette/struct.Lab.html\" title=\"struct palette::Lab\">Lab</a>&lt;Wp, T&gt;&gt; for <a class=\"struct\" href=\"palette/struct.Lab.html\" title=\"struct palette::Lab\">Lab</a>&lt;Wp, T&gt;<span class=\"where fmt-newline\">where\n    T: <a class=\"trait\" href=\"palette/trait.FloatComponent.html\" title=\"trait palette::FloatComponent\">FloatComponent</a> + <a class=\"trait\" href=\"approx/ulps_eq/trait.UlpsEq.html\" title=\"trait approx::ulps_eq::UlpsEq\">UlpsEq</a>,\n    T::<a class=\"associatedtype\" href=\"approx/abs_diff_eq/trait.AbsDiffEq.html#associatedtype.Epsilon\" title=\"type approx::abs_diff_eq::AbsDiffEq::Epsilon\">Epsilon</a>: <a class=\"trait\" href=\"https://doc.rust-lang.org/1.71.0/core/marker/trait.Copy.html\" title=\"trait core::marker::Copy\">Copy</a> + <a class=\"trait\" href=\"palette/trait.FloatComponent.html\" title=\"trait palette::FloatComponent\">FloatComponent</a>,\n    Wp: <a class=\"trait\" href=\"palette/white_point/trait.WhitePoint.html\" title=\"trait palette::white_point::WhitePoint\">WhitePoint</a> + <a class=\"trait\" href=\"https://doc.rust-lang.org/1.71.0/core/cmp/trait.PartialEq.html\" title=\"trait core::cmp::PartialEq\">PartialEq</a>,</span>"],["impl&lt;T&gt; <a class=\"trait\" href=\"approx/ulps_eq/trait.UlpsEq.html\" title=\"trait approx::ulps_eq::UlpsEq\">UlpsEq</a>&lt;<a class=\"struct\" href=\"palette/struct.Oklab.html\" title=\"struct palette::Oklab\">Oklab</a>&lt;T&gt;&gt; for <a class=\"struct\" href=\"palette/struct.Oklab.html\" title=\"struct palette::Oklab\">Oklab</a>&lt;T&gt;<span class=\"where fmt-newline\">where\n    T: <a class=\"trait\" href=\"palette/trait.FloatComponent.html\" title=\"trait palette::FloatComponent\">FloatComponent</a> + <a class=\"trait\" href=\"approx/ulps_eq/trait.UlpsEq.html\" title=\"trait approx::ulps_eq::UlpsEq\">UlpsEq</a>,\n    T::<a class=\"associatedtype\" href=\"approx/abs_diff_eq/trait.AbsDiffEq.html#associatedtype.Epsilon\" title=\"type approx::abs_diff_eq::AbsDiffEq::Epsilon\">Epsilon</a>: <a class=\"trait\" href=\"https://doc.rust-lang.org/1.71.0/core/marker/trait.Copy.html\" title=\"trait core::marker::Copy\">Copy</a> + <a class=\"trait\" href=\"palette/trait.FloatComponent.html\" title=\"trait palette::FloatComponent\">FloatComponent</a>,</span>"],["impl&lt;S, T&gt; <a class=\"trait\" href=\"approx/ulps_eq/trait.UlpsEq.html\" title=\"trait approx::ulps_eq::UlpsEq\">UlpsEq</a>&lt;<a class=\"struct\" href=\"palette/rgb/struct.Rgb.html\" title=\"struct palette::rgb::Rgb\">Rgb</a>&lt;S, T&gt;&gt; for <a class=\"struct\" href=\"palette/rgb/struct.Rgb.html\" title=\"struct palette::rgb::Rgb\">Rgb</a>&lt;S, T&gt;<span class=\"where fmt-newline\">where\n    T: <a class=\"trait\" href=\"palette/trait.Component.html\" title=\"trait palette::Component\">Component</a> + <a class=\"trait\" href=\"approx/ulps_eq/trait.UlpsEq.html\" title=\"trait approx::ulps_eq::UlpsEq\">UlpsEq</a>,\n    T::<a class=\"associatedtype\" href=\"approx/abs_diff_eq/trait.AbsDiffEq.html#associatedtype.Epsilon\" title=\"type approx::abs_diff_eq::AbsDiffEq::Epsilon\">Epsilon</a>: <a class=\"trait\" href=\"https://doc.rust-lang.org/1.71.0/core/marker/trait.Copy.html\" title=\"trait core::marker::Copy\">Copy</a>,\n    S: <a class=\"trait\" href=\"palette/rgb/trait.RgbStandard.html\" title=\"trait palette::rgb::RgbStandard\">RgbStandard</a> + <a class=\"trait\" href=\"https://doc.rust-lang.org/1.71.0/core/cmp/trait.PartialEq.html\" title=\"trait core::cmp::PartialEq\">PartialEq</a>,</span>"],["impl&lt;Wp, T&gt; <a class=\"trait\" href=\"approx/ulps_eq/trait.UlpsEq.html\" title=\"trait approx::ulps_eq::UlpsEq\">UlpsEq</a>&lt;<a class=\"struct\" href=\"palette/struct.Lch.html\" title=\"struct palette::Lch\">Lch</a>&lt;Wp, T&gt;&gt; for <a class=\"struct\" href=\"palette/struct.Lch.html\" title=\"struct palette::Lch\">Lch</a>&lt;Wp, T&gt;<span class=\"where fmt-newline\">where\n    T: <a class=\"trait\" href=\"palette/trait.FloatComponent.html\" title=\"trait palette::FloatComponent\">FloatComponent</a> + <a class=\"trait\" href=\"approx/ulps_eq/trait.UlpsEq.html\" title=\"trait approx::ulps_eq::UlpsEq\">UlpsEq</a>,\n    T::<a class=\"associatedtype\" href=\"approx/abs_diff_eq/trait.AbsDiffEq.html#associatedtype.Epsilon\" title=\"type approx::abs_diff_eq::AbsDiffEq::Epsilon\">Epsilon</a>: <a class=\"trait\" href=\"https://doc.rust-lang.org/1.71.0/core/marker/trait.Copy.html\" title=\"trait core::marker::Copy\">Copy</a> + <a class=\"trait\" href=\"palette/trait.FloatComponent.html\" title=\"trait palette::FloatComponent\">FloatComponent</a>,\n    Wp: <a class=\"trait\" href=\"palette/white_point/trait.WhitePoint.html\" title=\"trait palette::white_point::WhitePoint\">WhitePoint</a> + <a class=\"trait\" href=\"https://doc.rust-lang.org/1.71.0/core/cmp/trait.PartialEq.html\" title=\"trait core::cmp::PartialEq\">PartialEq</a>,</span>"],["impl&lt;C, T&gt; <a class=\"trait\" href=\"approx/ulps_eq/trait.UlpsEq.html\" title=\"trait approx::ulps_eq::UlpsEq\">UlpsEq</a>&lt;<a class=\"struct\" href=\"palette/blend/struct.PreAlpha.html\" title=\"struct palette::blend::PreAlpha\">PreAlpha</a>&lt;C, T&gt;&gt; for <a class=\"struct\" href=\"palette/blend/struct.PreAlpha.html\" title=\"struct palette::blend::PreAlpha\">PreAlpha</a>&lt;C, T&gt;<span class=\"where fmt-newline\">where\n    C: <a class=\"trait\" href=\"approx/ulps_eq/trait.UlpsEq.html\" title=\"trait approx::ulps_eq::UlpsEq\">UlpsEq</a>&lt;Epsilon = T::<a class=\"associatedtype\" href=\"approx/abs_diff_eq/trait.AbsDiffEq.html#associatedtype.Epsilon\" title=\"type approx::abs_diff_eq::AbsDiffEq::Epsilon\">Epsilon</a>&gt;,\n    T: <a class=\"trait\" href=\"approx/ulps_eq/trait.UlpsEq.html\" title=\"trait approx::ulps_eq::UlpsEq\">UlpsEq</a> + <a class=\"trait\" href=\"palette/float/trait.Float.html\" title=\"trait palette::float::Float\">Float</a>,\n    T::<a class=\"associatedtype\" href=\"approx/abs_diff_eq/trait.AbsDiffEq.html#associatedtype.Epsilon\" title=\"type approx::abs_diff_eq::AbsDiffEq::Epsilon\">Epsilon</a>: <a class=\"trait\" href=\"https://doc.rust-lang.org/1.71.0/core/marker/trait.Copy.html\" title=\"trait core::marker::Copy\">Copy</a>,</span>"],["impl&lt;Wp, T&gt; <a class=\"trait\" href=\"approx/ulps_eq/trait.UlpsEq.html\" title=\"trait approx::ulps_eq::UlpsEq\">UlpsEq</a>&lt;<a class=\"struct\" href=\"palette/struct.Lchuv.html\" title=\"struct palette::Lchuv\">Lchuv</a>&lt;Wp, T&gt;&gt; for <a class=\"struct\" href=\"palette/struct.Lchuv.html\" title=\"struct palette::Lchuv\">Lchuv</a>&lt;Wp, T&gt;<span class=\"where fmt-newline\">where\n    T: <a class=\"trait\" href=\"palette/trait.FloatComponent.html\" title=\"trait palette::FloatComponent\">FloatComponent</a> + <a class=\"trait\" href=\"approx/ulps_eq/trait.UlpsEq.html\" title=\"trait approx::ulps_eq::UlpsEq\">UlpsEq</a>,\n    T::<a class=\"associatedtype\" href=\"approx/abs_diff_eq/trait.AbsDiffEq.html#associatedtype.Epsilon\" title=\"type approx::abs_diff_eq::AbsDiffEq::Epsilon\">Epsilon</a>: <a class=\"trait\" href=\"https://doc.rust-lang.org/1.71.0/core/marker/trait.Copy.html\" title=\"trait core::marker::Copy\">Copy</a> + <a class=\"trait\" href=\"palette/trait.FloatComponent.html\" title=\"trait palette::FloatComponent\">FloatComponent</a>,\n    Wp: <a class=\"trait\" href=\"palette/white_point/trait.WhitePoint.html\" title=\"trait palette::white_point::WhitePoint\">WhitePoint</a> + <a class=\"trait\" href=\"https://doc.rust-lang.org/1.71.0/core/cmp/trait.PartialEq.html\" title=\"trait core::cmp::PartialEq\">PartialEq</a>,</span>"],["impl&lt;Wp, T&gt; <a class=\"trait\" href=\"approx/ulps_eq/trait.UlpsEq.html\" title=\"trait approx::ulps_eq::UlpsEq\">UlpsEq</a>&lt;<a class=\"struct\" href=\"palette/struct.Luv.html\" title=\"struct palette::Luv\">Luv</a>&lt;Wp, T&gt;&gt; for <a class=\"struct\" href=\"palette/struct.Luv.html\" title=\"struct palette::Luv\">Luv</a>&lt;Wp, T&gt;<span class=\"where fmt-newline\">where\n    T: <a class=\"trait\" href=\"palette/trait.FloatComponent.html\" title=\"trait palette::FloatComponent\">FloatComponent</a> + <a class=\"trait\" href=\"approx/ulps_eq/trait.UlpsEq.html\" title=\"trait approx::ulps_eq::UlpsEq\">UlpsEq</a>,\n    T::<a class=\"associatedtype\" href=\"approx/abs_diff_eq/trait.AbsDiffEq.html#associatedtype.Epsilon\" title=\"type approx::abs_diff_eq::AbsDiffEq::Epsilon\">Epsilon</a>: <a class=\"trait\" href=\"https://doc.rust-lang.org/1.71.0/core/marker/trait.Copy.html\" title=\"trait core::marker::Copy\">Copy</a> + <a class=\"trait\" href=\"palette/trait.FloatComponent.html\" title=\"trait palette::FloatComponent\">FloatComponent</a>,\n    Wp: <a class=\"trait\" href=\"palette/white_point/trait.WhitePoint.html\" title=\"trait palette::white_point::WhitePoint\">WhitePoint</a> + <a class=\"trait\" href=\"https://doc.rust-lang.org/1.71.0/core/cmp/trait.PartialEq.html\" title=\"trait core::cmp::PartialEq\">PartialEq</a>,</span>"],["impl&lt;S, T&gt; <a class=\"trait\" href=\"approx/ulps_eq/trait.UlpsEq.html\" title=\"trait approx::ulps_eq::UlpsEq\">UlpsEq</a>&lt;<a class=\"struct\" href=\"palette/luma/struct.Luma.html\" title=\"struct palette::luma::Luma\">Luma</a>&lt;S, T&gt;&gt; for <a class=\"struct\" href=\"palette/luma/struct.Luma.html\" title=\"struct palette::luma::Luma\">Luma</a>&lt;S, T&gt;<span class=\"where fmt-newline\">where\n    T: <a class=\"trait\" href=\"palette/trait.Component.html\" title=\"trait palette::Component\">Component</a> + <a class=\"trait\" href=\"approx/ulps_eq/trait.UlpsEq.html\" title=\"trait approx::ulps_eq::UlpsEq\">UlpsEq</a>,\n    T::<a class=\"associatedtype\" href=\"approx/abs_diff_eq/trait.AbsDiffEq.html#associatedtype.Epsilon\" title=\"type approx::abs_diff_eq::AbsDiffEq::Epsilon\">Epsilon</a>: <a class=\"trait\" href=\"https://doc.rust-lang.org/1.71.0/core/marker/trait.Copy.html\" title=\"trait core::marker::Copy\">Copy</a>,\n    S: <a class=\"trait\" href=\"palette/luma/trait.LumaStandard.html\" title=\"trait palette::luma::LumaStandard\">LumaStandard</a> + <a class=\"trait\" href=\"https://doc.rust-lang.org/1.71.0/core/cmp/trait.PartialEq.html\" title=\"trait core::cmp::PartialEq\">PartialEq</a>,</span>"],["impl&lt;S, T&gt; <a class=\"trait\" href=\"approx/ulps_eq/trait.UlpsEq.html\" title=\"trait approx::ulps_eq::UlpsEq\">UlpsEq</a>&lt;<a class=\"struct\" href=\"palette/struct.Hwb.html\" title=\"struct palette::Hwb\">Hwb</a>&lt;S, T&gt;&gt; for <a class=\"struct\" href=\"palette/struct.Hwb.html\" title=\"struct palette::Hwb\">Hwb</a>&lt;S, T&gt;<span class=\"where fmt-newline\">where\n    T: <a class=\"trait\" href=\"palette/trait.FloatComponent.html\" title=\"trait palette::FloatComponent\">FloatComponent</a> + <a class=\"trait\" href=\"approx/ulps_eq/trait.UlpsEq.html\" title=\"trait approx::ulps_eq::UlpsEq\">UlpsEq</a>,\n    T::<a class=\"associatedtype\" href=\"approx/abs_diff_eq/trait.AbsDiffEq.html#associatedtype.Epsilon\" title=\"type approx::abs_diff_eq::AbsDiffEq::Epsilon\">Epsilon</a>: <a class=\"trait\" href=\"https://doc.rust-lang.org/1.71.0/core/marker/trait.Copy.html\" title=\"trait core::marker::Copy\">Copy</a> + <a class=\"trait\" href=\"palette/float/trait.Float.html\" title=\"trait palette::float::Float\">Float</a> + <a class=\"trait\" href=\"palette/trait.FromF64.html\" title=\"trait palette::FromF64\">FromF64</a>,\n    S: <a class=\"trait\" href=\"palette/rgb/trait.RgbStandard.html\" title=\"trait palette::rgb::RgbStandard\">RgbStandard</a> + <a class=\"trait\" href=\"https://doc.rust-lang.org/1.71.0/core/cmp/trait.PartialEq.html\" title=\"trait core::cmp::PartialEq\">PartialEq</a>,</span>"],["impl&lt;Wp, T&gt; <a class=\"trait\" href=\"approx/ulps_eq/trait.UlpsEq.html\" title=\"trait approx::ulps_eq::UlpsEq\">UlpsEq</a>&lt;<a class=\"struct\" href=\"palette/struct.Yxy.html\" title=\"struct palette::Yxy\">Yxy</a>&lt;Wp, T&gt;&gt; for <a class=\"struct\" href=\"palette/struct.Yxy.html\" title=\"struct palette::Yxy\">Yxy</a>&lt;Wp, T&gt;<span class=\"where fmt-newline\">where\n    T: <a class=\"trait\" href=\"palette/trait.FloatComponent.html\" title=\"trait palette::FloatComponent\">FloatComponent</a> + <a class=\"trait\" href=\"approx/ulps_eq/trait.UlpsEq.html\" title=\"trait approx::ulps_eq::UlpsEq\">UlpsEq</a>,\n    T::<a class=\"associatedtype\" href=\"approx/abs_diff_eq/trait.AbsDiffEq.html#associatedtype.Epsilon\" title=\"type approx::abs_diff_eq::AbsDiffEq::Epsilon\">Epsilon</a>: <a class=\"trait\" href=\"https://doc.rust-lang.org/1.71.0/core/marker/trait.Copy.html\" title=\"trait core::marker::Copy\">Copy</a> + <a class=\"trait\" href=\"palette/trait.FloatComponent.html\" title=\"trait palette::FloatComponent\">FloatComponent</a>,\n    Wp: <a class=\"trait\" href=\"palette/white_point/trait.WhitePoint.html\" title=\"trait palette::white_point::WhitePoint\">WhitePoint</a> + <a class=\"trait\" href=\"https://doc.rust-lang.org/1.71.0/core/cmp/trait.PartialEq.html\" title=\"trait core::cmp::PartialEq\">PartialEq</a>,</span>"],["impl&lt;T: <a class=\"trait\" href=\"palette/float/trait.Float.html\" title=\"trait palette::float::Float\">Float</a> + <a class=\"trait\" href=\"palette/trait.FromF64.html\" title=\"trait palette::FromF64\">FromF64</a> + <a class=\"trait\" href=\"approx/ulps_eq/trait.UlpsEq.html\" title=\"trait approx::ulps_eq::UlpsEq\">UlpsEq</a>&gt; <a class=\"trait\" href=\"approx/ulps_eq/trait.UlpsEq.html\" title=\"trait approx::ulps_eq::UlpsEq\">UlpsEq</a>&lt;<a class=\"struct\" href=\"palette/struct.LuvHue.html\" title=\"struct palette::LuvHue\">LuvHue</a>&lt;T&gt;&gt; for <a class=\"struct\" href=\"palette/struct.LuvHue.html\" title=\"struct palette::LuvHue\">LuvHue</a>&lt;T&gt;<span class=\"where fmt-newline\">where\n    T::<a class=\"associatedtype\" href=\"approx/abs_diff_eq/trait.AbsDiffEq.html#associatedtype.Epsilon\" title=\"type approx::abs_diff_eq::AbsDiffEq::Epsilon\">Epsilon</a>: <a class=\"trait\" href=\"palette/float/trait.Float.html\" title=\"trait palette::float::Float\">Float</a> + <a class=\"trait\" href=\"palette/trait.FromF64.html\" title=\"trait palette::FromF64\">FromF64</a>,</span>"],["impl&lt;S, T&gt; <a class=\"trait\" href=\"approx/ulps_eq/trait.UlpsEq.html\" title=\"trait approx::ulps_eq::UlpsEq\">UlpsEq</a>&lt;<a class=\"struct\" href=\"palette/struct.Hsv.html\" title=\"struct palette::Hsv\">Hsv</a>&lt;S, T&gt;&gt; for <a class=\"struct\" href=\"palette/struct.Hsv.html\" title=\"struct palette::Hsv\">Hsv</a>&lt;S, T&gt;<span class=\"where fmt-newline\">where\n    T: <a class=\"trait\" href=\"palette/trait.FloatComponent.html\" title=\"trait palette::FloatComponent\">FloatComponent</a> + <a class=\"trait\" href=\"approx/ulps_eq/trait.UlpsEq.html\" title=\"trait approx::ulps_eq::UlpsEq\">UlpsEq</a>,\n    T::<a class=\"associatedtype\" href=\"approx/abs_diff_eq/trait.AbsDiffEq.html#associatedtype.Epsilon\" title=\"type approx::abs_diff_eq::AbsDiffEq::Epsilon\">Epsilon</a>: <a class=\"trait\" href=\"https://doc.rust-lang.org/1.71.0/core/marker/trait.Copy.html\" title=\"trait core::marker::Copy\">Copy</a> + <a class=\"trait\" href=\"palette/float/trait.Float.html\" title=\"trait palette::float::Float\">Float</a> + <a class=\"trait\" href=\"palette/trait.FromF64.html\" title=\"trait palette::FromF64\">FromF64</a>,\n    S: <a class=\"trait\" href=\"palette/rgb/trait.RgbStandard.html\" title=\"trait palette::rgb::RgbStandard\">RgbStandard</a> + <a class=\"trait\" href=\"https://doc.rust-lang.org/1.71.0/core/cmp/trait.PartialEq.html\" title=\"trait core::cmp::PartialEq\">PartialEq</a>,</span>"],["impl&lt;T: <a class=\"trait\" href=\"palette/float/trait.Float.html\" title=\"trait palette::float::Float\">Float</a> + <a class=\"trait\" href=\"palette/trait.FromF64.html\" title=\"trait palette::FromF64\">FromF64</a> + <a class=\"trait\" href=\"approx/ulps_eq/trait.UlpsEq.html\" title=\"trait approx::ulps_eq::UlpsEq\">UlpsEq</a>&gt; <a class=\"trait\" href=\"approx/ulps_eq/trait.UlpsEq.html\" title=\"trait approx::ulps_eq::UlpsEq\">UlpsEq</a>&lt;<a class=\"struct\" href=\"palette/struct.OklabHue.html\" title=\"struct palette::OklabHue\">OklabHue</a>&lt;T&gt;&gt; for <a class=\"struct\" href=\"palette/struct.OklabHue.html\" title=\"struct palette::OklabHue\">OklabHue</a>&lt;T&gt;<span class=\"where fmt-newline\">where\n    T::<a class=\"associatedtype\" href=\"approx/abs_diff_eq/trait.AbsDiffEq.html#associatedtype.Epsilon\" title=\"type approx::abs_diff_eq::AbsDiffEq::Epsilon\">Epsilon</a>: <a class=\"trait\" href=\"palette/float/trait.Float.html\" title=\"trait palette::float::Float\">Float</a> + <a class=\"trait\" href=\"palette/trait.FromF64.html\" title=\"trait palette::FromF64\">FromF64</a>,</span>"],["impl&lt;C, T&gt; <a class=\"trait\" href=\"approx/ulps_eq/trait.UlpsEq.html\" title=\"trait approx::ulps_eq::UlpsEq\">UlpsEq</a>&lt;<a class=\"struct\" href=\"palette/struct.Alpha.html\" title=\"struct palette::Alpha\">Alpha</a>&lt;C, T&gt;&gt; for <a class=\"struct\" href=\"palette/struct.Alpha.html\" title=\"struct palette::Alpha\">Alpha</a>&lt;C, T&gt;<span class=\"where fmt-newline\">where\n    C: <a class=\"trait\" href=\"approx/ulps_eq/trait.UlpsEq.html\" title=\"trait approx::ulps_eq::UlpsEq\">UlpsEq</a>&lt;Epsilon = T::<a class=\"associatedtype\" href=\"approx/abs_diff_eq/trait.AbsDiffEq.html#associatedtype.Epsilon\" title=\"type approx::abs_diff_eq::AbsDiffEq::Epsilon\">Epsilon</a>&gt;,\n    T: <a class=\"trait\" href=\"approx/ulps_eq/trait.UlpsEq.html\" title=\"trait approx::ulps_eq::UlpsEq\">UlpsEq</a>,\n    T::<a class=\"associatedtype\" href=\"approx/abs_diff_eq/trait.AbsDiffEq.html#associatedtype.Epsilon\" title=\"type approx::abs_diff_eq::AbsDiffEq::Epsilon\">Epsilon</a>: <a class=\"trait\" href=\"https://doc.rust-lang.org/1.71.0/core/clone/trait.Clone.html\" title=\"trait core::clone::Clone\">Clone</a>,</span>"],["impl&lt;T: <a class=\"trait\" href=\"palette/float/trait.Float.html\" title=\"trait palette::float::Float\">Float</a> + <a class=\"trait\" href=\"palette/trait.FromF64.html\" title=\"trait palette::FromF64\">FromF64</a> + <a class=\"trait\" href=\"approx/ulps_eq/trait.UlpsEq.html\" title=\"trait approx::ulps_eq::UlpsEq\">UlpsEq</a>&gt; <a class=\"trait\" href=\"approx/ulps_eq/trait.UlpsEq.html\" title=\"trait approx::ulps_eq::UlpsEq\">UlpsEq</a>&lt;<a class=\"struct\" href=\"palette/struct.LabHue.html\" title=\"struct palette::LabHue\">LabHue</a>&lt;T&gt;&gt; for <a class=\"struct\" href=\"palette/struct.LabHue.html\" title=\"struct palette::LabHue\">LabHue</a>&lt;T&gt;<span class=\"where fmt-newline\">where\n    T::<a class=\"associatedtype\" href=\"approx/abs_diff_eq/trait.AbsDiffEq.html#associatedtype.Epsilon\" title=\"type approx::abs_diff_eq::AbsDiffEq::Epsilon\">Epsilon</a>: <a class=\"trait\" href=\"palette/float/trait.Float.html\" title=\"trait palette::float::Float\">Float</a> + <a class=\"trait\" href=\"palette/trait.FromF64.html\" title=\"trait palette::FromF64\">FromF64</a>,</span>"]]
};if (window.register_implementors) {window.register_implementors(implementors);} else {window.pending_implementors = implementors;}})()